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
        std::cerr << "Failed to open driver" << std::endl;
        return;
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

void driver_interface::clear_mmap()
{
    if (va_mmap_)
    {
        munmap(va_mmap_, TOTAL_SIZE);
        va_mmap_ = NULL;
    }
}

void driver_interface::init_mmap(eo_as_mem_map &memory_data)
{
    va_mmap_ = mmap(NULL, TOTAL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, driverHandle_, 0);
    if (va_mmap_ == MAP_FAILED)
    {
        std::cerr << "Invalid driver memory" << std::endl;
        return;
    }

    for (uint32_t channel = 0; channel < MAX_NUM_CHANNELS; ++channel)
    {
        for (uint32_t descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; ++descriptor)
        {
            uint64_t index = (channel * MAX_NUM_DESCRIPTORS) + descriptor;
            uint8_t *va_mmap_b8 = (uint8_t *)va_mmap_;
            memory_data.chans[channel].descs[descriptor].buf_va = reinterpret_cast<uint64_t>(va_mmap_b8 + (DESCRIPTOR_BUF_SIZE * index));
        }
    }
}

bool driver_interface::send_ioctl(unsigned long ioctlCode, void *arg)
{
    if (driverHandle_ < 0)
    {
        std::cerr << "Invalid driver handle" << std::endl;
        return false;
    }

    return ioctl(driverHandle_, ioctlCode, arg) >= 0;
}

void driver_interface::write_register(uint32_t bar, uint64_t registerOffset, uint32_t value)
{
    struct eo_as_registry_params info = {registerOffset, value, bar};

    std::cout << "bar " << info.bar << " addr " << info.address << " value " << info.value << std::endl;

    if (!send_ioctl(EO_AS_IOC_SET_DMA_REG, &info))
    {
        std::cerr << "Failed to write to register" << std::endl;
        return;
    }
}

uint32_t driver_interface::read_register(uint32_t bar, uint64_t registerOffset)
{
    struct eo_as_registry_params info = {registerOffset, 0, bar};

    std::cout << "bar " << info.bar << " addr " << info.address << " value " << info.value << std::endl;

    if (!send_ioctl(EO_AS_IOC_GET_DMA_REG, &info))
    {
        std::cerr << "Failed to read from register" << std::endl;
        return 0;
    }
    return info.value;
}

int driver_interface::get_handle() const
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
            std::cerr << "Failed to call EO_AS_IOC_DMA_PARAMS_GET" << std::endl;
            return;
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
                    std::cerr << "Cannot create event for channel " << std::to_string(channel) << std::endl;
                    return;
                }

                std::cout << "Created event: " << eventName << " for channel = " << channel << " and descriptor = " << descriptor << " eventfd " << sharedEventHandle_[index] << std::endl;
            }
        }

        if (!send_ioctl(EO_AS_IOC_EVENT_HANDLE_SET, sharedEventHandle_))
        {
            std::cerr << "Failed to call EO_AS_IOC_EVENT_HANDLE_SET" << std::endl;
            return;
        }

        if (!send_ioctl(EO_AS_IOC_EVENT_HANDLE_GET, &eventData))
        {
            std::cerr << "Failed to call EO_AS_IOC_EVENT_HANDLE_GET" << std::endl;
            return;
        }

        if (!send_ioctl(EO_AS_IOC_MEM_MAP_GET, &memoryData))
        {
            std::cerr << "Failed to call EO_AS_IOC_MEM_MAP_GET" << std::endl;
            return;
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
            std::cerr << "Invalid channel parameter" << std::endl;
            return;
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
        uint32_t InterruptStatusValue = read_register(0, trans_form_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS));
        write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_DATA), 0x00FF);

        for (int channel = 0; channel < MAX_NUM_CHANNELS; channel++)
        {
            write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_CONTROL) + 0x40 * channel, 0x00000000);
            write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_NUMBER) + 0x40 * channel, MAX_NUM_DESCRIPTORS);

            for (int descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; descriptor++)
            {
                int index = (channel * MAX_NUM_DESCRIPTORS) + descriptor;

                printf("Mapped struct: buf_va = 0x%lx, buf_pa = 0x%lx\n", data.chans[channel].descs[descriptor].buf_va, data.chans[channel].descs[descriptor].buf_pa);

                uint32_t PaLowValue = data.chans[channel].descs[descriptor].buf_pa & 0xFFFFFFFF;
                uint32_t PaHighValue = (data.chans[channel].descs[descriptor].buf_pa >> 32) & 0xFFFFFFFF;

                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor, PaLowValue);
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0x4, PaHighValue);

                uint32_t DmaBufferSize = std::min(startDmaConfiguration.start_dma_channels[channel].start_dma_descriptors[descriptor].dma_descriptor_buffer_size, static_cast<uint32_t>(DESCRIPTOR_BUF_SIZE));
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0x8, DmaBufferSize | (1 << 31));

                uint32_t IsDescriptorInterruptEnable = std::min(startDmaConfiguration.start_dma_channels[channel].start_dma_descriptors[descriptor].is_descriptor_interrupt_enable, static_cast<uint32_t>(true));
                write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE) + 0x400 * channel + 0x10 * descriptor + 0xc, IsDescriptorInterruptEnable);
            }
        }

        write_register(0, trans_form_fpga_address(DEVICE_GLOBAL_DMA_PPS_TRIGER), 0x00000000);
    };

    configuration_start_DMA();
}
