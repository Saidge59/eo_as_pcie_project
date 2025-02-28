#include "eo_as_interface_lib.h"
#include <sys/eventfd.h>
#include <sstream>
#include <sys/mman.h>

std::mutex driver_interface::g_mutex;

driver_interface::driver_interface(const char *devicePath)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    driverHandle_ = open(devicePath, O_RDWR);
    if (driverHandle_ < 0)
    {
        throw std::runtime_error("Failed to open driver");
    }
}

driver_interface::~driver_interface()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (driverHandle_ >= 0)
    {
        close(driverHandle_);
        driverHandle_ = -1;
    }
}

bool driver_interface::send_ioctl(unsigned long ioctlCode, void *arg)
{
    if (driverHandle_ < 0)
    {
        throw std::runtime_error("Invalid driver handle");
    }

    return ioctl(driverHandle_, ioctlCode, arg) >= 0;
}

void driver_interface::write_register(uint32_t bar, uint64_t registerOffset, uint32_t value)
{
    struct eo_as_registry_params info = {registerOffset, value, bar};

    std::cout << "bar " << info.bar << " addr " << info.address << " value " << info.value << std::endl;

    if (!send_ioctl(EO_AS_IOC_SET_DMA_REG, &info))
    {
        throw std::runtime_error("Failed to write to register");
    }
}

uint32_t driver_interface::read_register(uint32_t bar, uint64_t registerOffset)
{
    struct eo_as_registry_params info = {registerOffset, 0, bar};

    std::cout << "bar " << info.bar << " addr " << info.address << " value " << info.value << std::endl;

    if (!send_ioctl(EO_AS_IOC_GET_DMA_REG, &info))
    {
        throw std::runtime_error("Failed to read from register");
    }
    return info.value;
}

int driver_interface::GetHandle() const
{
    return driverHandle_;
}

void driver_interface::read_DMA_memory_map_and_event_handles(struct eo_as_dma_params &dmaParam, struct eo_as_mem_map &memoryData, struct eo_as_event_data &eventData)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    auto read_memory_map = [&]()
    {
        if (!send_ioctl(EO_AS_IOC_DMA_PARAMS_GET, &dmaParam))
        {
            throw std::runtime_error("Failed to call EO_AS_IOC_DMA_PARAMS_GET");
        }

        pid_t processID = getpid();
        for (uint32_t channel = 0; channel < dmaParam.dma_chan_count; ++channel)
        {
            for (uint32_t descriptor = 0; descriptor < dmaParam.dma_desc_count; ++descriptor)
            {
                std::stringstream ss;
                ss << "/dev/shm/dma_event_" << processID << "_C" << channel << "_D" << descriptor;
                std::string eventName = ss.str();

                int index = channel * dmaParam.dma_desc_count + descriptor;
                sharedEventHandle_[index] = eventfd(0, EFD_NONBLOCK);
                if (sharedEventHandle_[index] == -1)
                {
                    throw std::runtime_error("Cannot create event for channel " + std::to_string(channel));
                }

                std::cout << "Created event: " << eventName << " for channel = " << channel << " and descriptor = " << descriptor << " eventfd " << sharedEventHandle_[index] << std::endl;
            }
        }

        if (!send_ioctl(EO_AS_IOC_EVENT_HANDLE_SET, sharedEventHandle_))
        {
            throw std::runtime_error("Failed to call EO_AS_IOC_EVENT_HANDLE_SET");
        }

        if (!send_ioctl(EO_AS_IOC_EVENT_HANDLE_GET, &eventData))
        {
            throw std::runtime_error("Failed to call EO_AS_IOC_EVENT_HANDLE_GET");
        }

        if (!send_ioctl(EO_AS_IOC_MEM_MAP_GET, &memoryData))
        {
            throw std::runtime_error("Failed to call EO_AS_IOC_MEM_MAP_GET");
        }
    };

    read_memory_map();
}

void driver_interface::start_stop_DMA_channel(uint8_t channel, bool isStartDmaChannel, bool isCycle)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    auto start_stop_DMA_channel = [&]()
    {
        if (channel >= MAX_NUM_CHANNELS)
        {
            throw std::runtime_error("Invalid channel parameter");
        }

        uint32_t DmaControlValue = isStartDmaChannel ? 0x00000003 : 0x00000000;
        if (isCycle)
        {
            DmaControlValue |= 0x00000008;
        }

        std::cout << "StartStopDmaChannel" << std::endl;
        write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_CONTROL) + 0x40 * channel, DmaControlValue);
    };

    start_stop_DMA_channel();
}

void driver_interface::start_stop_DMA_global(bool isStartDmaGlobal, bool isRx)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    auto start_stop_DMA_global = [&]()
    {
        uint32_t value = isStartDmaGlobal ? 0x00000001 : 0x00000000;
        uint64_t address = isRx ? DEVICE_GLOBAL_DMA_ENABLE_FPGA_DATA : DEVICE_GLOBAL_DMA_MAX_DMA_CHANELS_DATA;

        write_register(0, trans_form_fpga_address(address), value);
    };

    start_stop_DMA_global();
}

void driver_interface::start_DMA_configure(struct global_start_dma_configuration &startDmaConfiguration, struct eo_as_mem_map &data)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    auto configuration_start_DMA = [&]()
    {
        struct eo_as_dma_desc *dma_desc = (struct eo_as_dma_desc *)mmap(NULL, (sizeof(struct eo_as_dma_desc) * MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS), PROT_READ | PROT_WRITE, MAP_SHARED, driverHandle_, 0);
        if (dma_desc == MAP_FAILED)
        {
            throw std::runtime_error("Invalid driver memory");
        }

        uint32_t InterruptStatusValue = read_register(0, trans_form_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS));
        write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_DATA), 0x00FF);

        for (int channel = 0; channel < MAX_NUM_CHANNELS; channel++)
        {
            write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_CONTROL) + 0x40 * channel, 0x00000000);
            write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_NUMBER) + 0x40 * channel, MAX_NUM_DESCRIPTORS);

            for (int descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; descriptor++)
            {
                int index = (channel * MAX_NUM_DESCRIPTORS) + descriptor;

                struct eo_as_dma_desc *current_dma_desc = dma_desc + index;
                data.chans[channel].descs[descriptor].buf_va = current_dma_desc->buf_va;
                data.chans[channel].descs[descriptor].buf_pa = current_dma_desc->buf_pa;
                printf("Mapped struct: buf_va = 0x%lx, buf_pa = 0x%lx\n", data.chans[channel].descs[descriptor].buf_va, data.chans[channel].descs[descriptor].buf_pa);

                uint32_t PaLowValue = data.chans[channel].descs[descriptor].buf_pa & 0xFFFFFFFF;
                uint32_t PaHighValue = (data.chans[channel].descs[descriptor].buf_pa >> 32) & 0xFFFFFFFF;

                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor, PaLowValue);
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0x4, PaHighValue);

                uint32_t DmaBufferSize = std::min(startDmaConfiguration.start_dma_channels[channel].start_dma_descriptors[descriptor].dma_descriptor_buffer_size, static_cast<uint32_t>(DESCRIPTOR_BUFFER_SIZE));
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0x8, DmaBufferSize | (1 << 31));

                uint32_t IsDescriptorInterruptEnable = std::min(startDmaConfiguration.start_dma_channels[channel].start_dma_descriptors[descriptor].is_descriptor_interrupt_enable, static_cast<uint32_t>(true));
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0xc, IsDescriptorInterruptEnable);
            }
        }

        if (dma_desc)
        {
            munmap(dma_desc, sizeof(struct eo_as_dma_desc));
            dma_desc = NULL;
        }

        write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_PPS_TRIGER), 0x00000000);
    };

    configuration_start_DMA();
}
