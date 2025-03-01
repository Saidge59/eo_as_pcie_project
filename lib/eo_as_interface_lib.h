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

/**
 * @brief Class providing an interface to interact with the FPGA driver.
 *
 * Manages communication with the driver through IOCTLs, register access, and DMA operations.
 */
class driver_interface
{
public:
    /**
     * @brief Constructs the driver interface with a specified device path.
     *
     * @param devicePath Path to the device file (e.g., "/dev/eo_as_fpga_driver").
     */
    explicit driver_interface(const char *devicePath);

    /**
     * @brief Destructor for the driver interface.
     *
     * Closes the driver handle and releases resources.
     */
    ~driver_interface();

    /**
     * @brief Clear memory map.
     *
     * @param va_mmap Pointer to virtual address.
     */
    void clear_mmap(void *va_mmap);

    /**
     * @brief Sends an IOCTL command to the driver.
     *
     * @param ioctlCode The IOCTL command code to send.
     * @param inBuffer Pointer to the input buffer for the IOCTL.
     * @return True if the IOCTL was sent successfully, false otherwise.
     */
    bool send_ioctl(unsigned long ioctlCode, void *inBuffer);

    /**
     * @brief Writes a 32-bit value to an FPGA register.
     *
     * @param bar The BAR index to write to.
     * @param registerOffset Offset of the register within the BAR.
     * @param value The 32-bit value to write.
     */
    void write_register(uint32_t bar, uint64_t registerOffset, uint32_t value);

    /**
     * @brief Reads a 32-bit value from an FPGA register.
     *
     * @param bar The BAR index to read from.
     * @param registerOffset Offset of the register within the BAR.
     * @return The 32-bit value read from the register.
     */
    uint32_t read_register(uint32_t bar, uint64_t registerOffset);

    /**
     * @brief Gets the driver file handle.
     *
     * @return The file descriptor of the driver handle.
     */
    int get_handle() const;

    /**
     * @brief Reads DMA memory map and event handle data from the driver.
     *
     * @param dmaParam Reference to the DMA parameters structure to populate.
     * @param memoryData Reference to the memory map structure to populate.
     * @param eventData Reference to the event data structure to populate.
     */
    void read_DMA_memory_map_and_event_handles(struct eo_as_dma_params &dmaParam, struct eo_as_mem_map &memoryData, struct eo_as_event_data &eventData);

    /**
     * @brief Starts or stops a specific DMA channel.
     *
     * @param channel The DMA channel index.
     * @param isStartDmaChannel True to start, false to stop the channel.
     * @param isCycle True to enable cyclic mode, false otherwise.
     */
    void start_stop_DMA_channel(uint8_t channel, bool isStartDmaChannel, bool isCycle);

    /**
     * @brief Starts or stops global DMA operations.
     *
     * @param isStartDmaGlobal True to start, false to stop global DMA.
     * @param isRx True for receive mode, false for transmit mode.
     */
    void start_stop_DMA_global(bool isStartDmaGlobal, bool isRx);

    /**
     * @brief Configures and starts DMA with provided settings.
     *
     * @param startDmaConfiguration Reference to the global DMA configuration structure.
     * @param data Reference to the memory map structure for DMA data.
     * @param va_mmap Pointer to virtual address.
     */
    void start_DMA_configure(struct global_start_dma_configuration &startDmaConfiguration, struct eo_as_mem_map &data, void *&va_mmap);

private:
    int driverHandle_ = -1; ///< File descriptor for the driver.
    int sharedEventHandle_[MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS]; ///< Array of shared event handles.
    static std::mutex g_mutex; ///< Mutex for thread-safe operations.
};

#endif // DRIVER_INTERFACE_H
