/*
 * @file Public.h
 * @brief Constants and structures for IOCTL and DMA data transfer in Linux.
 *
 * This section defines various constants, macros, data types, and structures
 * used for IOCTL commands and DMA data transfer between the driver and user applications.
 *
 */
#ifndef PUBLIC_H
#define PUBLIC_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define DRV_VER 0x102

#define MAX_NUM_CHANNELS 8                        ///< Maximum number of DMA channels
#define MAX_NUM_CHANNELS_WITH_HEADER 8            ///< Maximum number of DMA channels for project with read size in header
#define MAX_NUM_DESCRIPTORS 6                     ///< Maximum number of DMA descriptors per channel
#define MAX_NUM_EVENTS_PER_DESCRIPTORS 3          ///< Maximum number of events per descriptor
#define DESCRIPTOR_BUF_SIZE (32ULL * 1024 * 1024) ///< Descriptor buffer size set to 1 GB
#define TOTAL_SIZE (MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS * DESCRIPTOR_BUF_SIZE)

#define ALIGN_X(value, x) (((value) + ((x) - 1)) & -(x)) ///< Align 'value' to the nearest multiple of 'x'
#define ALIGN_64(value) ALIGN_X((value), 64)             ///< Align 'value' to the nearest multiple of 64

#define TIMEOUT_MS_IOCTL 2000 ///< IOCTL call timeout in ms

#define DEVICE_GLOBAL_DRV_VER 0xb                         ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ENABLE 0x0003        ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS 0x0005        ///< Global status Interrupt register (FIFO occupancy)
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ACK 0x0004           ///< Global status Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_DATA 0x0006          ///< Global Interrupt register data (read before ACK)
#define DEVICE_GLOBAL_DMA_ENABLE_FPGA_DATA 0x0008         ///< Global DMA enable register data
#define DEVICE_GLOBAL_DMA_MAX_DMA_CHANELS_DATA 0x0009     ///< Global DMA max channels register data
#define DEVICE_GLOBAL_DMA_PPS_TRIGER 0x0017               ///< Global PPS triger bit 0: Enable, bit 1: Rising/Fall
#define DEVICE_GLOBAL_DMA_REG_CONTROL 0x0100              ///< Register address for DMA control operations.
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_NUMBER 0x0101   ///< Register address for querying the number of DMA descriptors.
#define DEVICE_GLOBAL_DMA_REG_GET_DESCRIPTOR_INDEX 0x0102 ///< Register address for obtaining the current descriptor index.
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE 0x0800    ///< Base address of the DMA descriptors table.

/**
 * @brief Transforms an FPGA address by shifting it left.
 *
 * Multiplies the input address by 4 by performing a left shift of 2 bits.
 *
 * @param address The address to transform.
 * @return The transformed address (shifted left by 2).
 */
static inline uint64_t trans_form_fpga_address(uint64_t address)
{
    return address << 2;
}

/**
 * @brief Structure for FPGA register access parameters.
 *
 * Contains the register offset, value, and BAR index for read/write operations.
 */
struct eo_as_registry_params
{
    uint64_t address;
    uint32_t value;
    uint32_t bar;
} __attribute__((packed));

/**
 * @brief Structure for DMA-related parameters.
 *
 * Defines the maximum counts and sizes for DMA descriptors and channels.
 */
struct eo_as_dma_params
{
    uint32_t dma_desc_count;
    uint32_t dma_buf_size;
    uint32_t dma_chan_count;
} __attribute__((packed));

/**
 * @brief Structure for a single DMA descriptor.
 *
 * Holds the virtual and physical addresses of a DMA buffer.
 */
struct eo_as_dma_desc
{
    uint64_t buf_va;
    uint64_t buf_pa;
} __attribute__((packed));

/**
 * @brief Structure for a DMA channel.
 *
 * Contains an array of DMA descriptors for a single channel.
 */
struct eo_as_dma_channel
{
    struct eo_as_dma_desc descs[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

/**
 * @brief Structure for memory mapping of DMA channels.
 *
 * Contains an array of DMA channels for memory mapping.
 */
struct eo_as_mem_map
{
    struct eo_as_dma_channel chans[MAX_NUM_CHANNELS];
} __attribute__((packed));

/**
 * @brief Structure for configuring a single DMA descriptor.
 *
 * Specifies the buffer size and interrupt enable flag for a DMA descriptor.
 */
struct start_dma_descriptors_configuration
{
    uint32_t dma_descriptor_buffer_size;
    uint32_t is_descriptor_interrupt_enable;
} __attribute__((packed));

/**
 * @brief Structure for configuring a DMA channel.
 *
 * Defines the number of descriptors and their configurations for a channel.
 */
struct start_dma_configuration
{
    uint32_t dma_descriptors_count;
    struct start_dma_descriptors_configuration start_dma_descriptors[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

/**
 * @brief Structure for global DMA configuration.
 *
 * Specifies the number of channels, start cycle, and per-channel configurations.
 */
struct global_start_dma_configuration
{
    uint32_t dma_channels_count;
    uint32_t start_cycle;
    struct start_dma_configuration start_dma_channels[MAX_NUM_CHANNELS];
} __attribute__((packed));

/**
 * @brief Structure for an event descriptor.
 *
 * Contains an event handle and associated eventfd context.
 */
struct eo_as_event_desc
{
    uint64_t event_handle; /* user-provided token */
    struct eventfd_ctx *event_ctx;
} __attribute__((packed));

/**
 * @brief Structure for event data of a single channel.
 *
 * Holds an array of event descriptors for a channel.
 */
struct eo_as_event_chan
{
    struct eo_as_event_desc events[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

/**
 * @brief Structure for event data across all channels.
 *
 * Contains an array of channel-specific event data.
 */
struct eo_as_event_data
{
    struct eo_as_event_chan chans[MAX_NUM_CHANNELS];
} __attribute__((packed));

#define EO_AS_IOC_MAGIC 0xB0
#define EO_AS_IOC_SET_DMA_REG _IOW(EO_AS_IOC_MAGIC, 0x01, struct eo_as_registry_params)
#define EO_AS_IOC_GET_DMA_REG _IOWR(EO_AS_IOC_MAGIC, 0x02, struct eo_as_registry_params)
#define EO_AS_IOC_GET_DMA_STATUS _IOR(EO_AS_IOC_MAGIC, 0x03, __u32)
#define EO_AS_IOC_DMA_PARAMS_GET _IOR(EO_AS_IOC_MAGIC, 0x04, struct eo_as_dma_params)
#define EO_AS_IOC_MEM_MAP_GET _IOR(EO_AS_IOC_MAGIC, 0x05, struct eo_as_mem_map)
#define EO_AS_IOC_EVENT_HANDLE_SET _IOW(EO_AS_IOC_MAGIC, 0x06, struct eo_as_event_data)
#define EO_AS_IOC_EVENT_HANDLE_GET _IOR(EO_AS_IOC_MAGIC, 0x07, struct eo_as_event_data)

#endif /* PUBLIC_H */
