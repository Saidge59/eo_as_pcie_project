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

///< Maximum number of channels and descriptors
#define MAX_NUM_CHANNELS 8                            ///< Maximum number of DMA channels
#define MAX_NUM_CHANNELS_WITH_HEADER 8                ///< Maximum number of DMA channels for project with read size in header
#define MAX_NUM_DESCRIPTORS 6                         ///< Maximum number of DMA descriptors per channel
#define MAX_NUM_EVENTS_PER_DESCRIPTORS 3              ///< Maximum number of events per descriptor
#define DESCRIPTOR_BUFFER_SIZE (32ULL * 1024 * 1024) ///< Descriptor buffer size set to 1 GB

///< COMMON MACROS
///< Alignment for X is power of 2
#define ALIGN_X(value, x) (((value) + ((x) - 1)) & -(x)) ///< Align 'value' to the nearest multiple of 'x'
#define ALIGN_64(value) ALIGN_X((value), 64)             ///< Align 'value' to the nearest multiple of 64

#define TIMEOUT_MS_IOCTL 2000 ///< IOCTL call timeout in ms

#define DEVICE_GLOBAL_DRV_VER 0xb                     ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ENABLE 0x0003    ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS 0x0005    ///< Global status Interrupt register (FIFO occupancy)
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ACK 0x0004       ///< Global status Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_DATA 0x0006      ///< Global Interrupt register data (read before ACK)
#define DEVICE_GLOBAL_DMA_ENABLE_FPGA_DATA 0x0008     ///< Global DMA enable register data
#define DEVICE_GLOBAL_DMA_MAX_DMA_CHANELS_DATA 0x0009 ///< Global DMA max channels register data
#define DEVICE_GLOBAL_DMA_PPS_TRIGER 0x0017           ///< Global PPS triger bit 0: Enable, bit 1: Rising/Fall

///< DMA control register address
#define DEVICE_GLOBAL_DMA_REG_CONTROL 0x0100 ///< Register address for DMA control operations.

///< DMA descriptors number register address
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_NUMBER 0x0101 ///< Register address for querying the number of DMA descriptors.

///< DMA get descriptor index register address
#define DEVICE_GLOBAL_DMA_REG_GET_DESCRIPTOR_INDEX 0x0102 ///< Register address for obtaining the current descriptor index.

///< DMA descriptors table base address
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE 0x0800 ///< Base address of the DMA descriptors table.

///< Function to transform FPGA address
static inline uint64_t trans_form_fpga_address(uint64_t address)
{
    return address << 2; ///< Left shift by 2 positions to multiply by 4
}

/*
 * Convert old Windows IOCTL_* definitions to Linux _IO, _IOR, _IOW, _IOWR
 * and rename fields to Linux style (e.g. "dma_desc_count").
 */

/* Registry-like operation for reading/writing a register */
struct eo_as_registry_params
{
    uint64_t address; /* register offset */
    uint32_t value;   /* read/write value */
    uint32_t bar;      /* which BAR? */
} __attribute__((packed));

/*
 * Replacing GLOBAL_DATA_DMA_PARAMETERS with Linux naming
 */
struct eo_as_dma_params
{
    uint32_t dma_desc_count; /* was DmaDescriptorsMaxCount */
    uint32_t dma_buf_size;   /* was DmaDescriptorMaxBufferSize */
    uint32_t dma_chan_count; /* was DmaChannelsMaxCount */
} __attribute__((packed));

struct eo_as_dma_desc
{
    uint64_t buf_va;
    uint64_t buf_pa;
} __attribute__((packed));

struct eo_as_dma_channel
{
    struct eo_as_dma_desc descs[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

struct eo_as_mem_map
{
    struct eo_as_dma_channel chans[MAX_NUM_CHANNELS];
} __attribute__((packed));

// GLOBAL_MEM_MAP_DATA_AND_EVENT_HANDLES
/**
 * @brief Configuration for starting a DMA descriptor.
 */
struct start_dma_descriptors_configuration
{
    uint32_t dma_descriptor_buffer_size;     ///< Size of the DMA descriptor buffer
    uint32_t is_descriptor_interrupt_enable; ///< Flag to enable interrupt for the descriptor
} __attribute__((packed));

/**
 * @brief Configuration for starting a DMA channel.
 */
struct start_dma_configuration
{
    uint32_t dma_descriptors_count;                                                 ///< Number of DMA descriptors in the channel
    struct start_dma_descriptors_configuration start_dma_descriptors[MAX_NUM_DESCRIPTORS]; ///< Array of DMA descriptor configurations
} __attribute__((packed));

struct global_start_dma_configuration
{
    uint32_t dma_channels_count;
    uint32_t start_cycle;
    struct start_dma_configuration start_dma_channels[MAX_NUM_CHANNELS];
} __attribute__((packed));

/*
 * If you previously had event-handle definitions, rename them likewise.
 * In Linux, there's no direct concept of "HANDLE", so you might store a user token:
 */
struct eo_as_event_desc
{
    uint64_t event_handle; /* user-provided token */
    struct eventfd_ctx *event_ctx;
} __attribute__((packed));

struct eo_as_event_chan
{
    struct eo_as_event_desc events[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

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
