#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <cstring>
#include "public.h"

class driver_interface
{
public:
    explicit driver_interface(const char *devicePath);
    ~driver_interface();

    bool send_ioctl(unsigned long ioctlCode, void *inBuffer);

    void write_register(uint8_t bar, uint64_t registerOffset, uint32_t value);

    uint32_t read_register(uint8_t bar, uint64_t registerOffset);

    int GetHandle() const;

    void read_DMA_memory_map_and_event_handles(struct eo_as_dma_params &dmaParam, struct eo_as_mem_map &memoryData, struct eo_as_event_data &eventData);
    void start_stop_DMA_channel(uint8_t channel, bool isStartDmaChannel, bool isCycle);
    void start_stop_DMA_global(bool isStartDmaGlobal, bool isRx);
    void start_DMA_configure(struct global_start_dma_configuration &startDmaConfiguration, struct eo_as_mem_map &data);

private:
    int driverHandle_ = -1;
    int sharedEventHandle_[MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS];
    static std::mutex g_mutex;
};

#endif // DRIVER_INTERFACE_H
