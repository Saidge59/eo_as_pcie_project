#ifndef EO_AS_IOCTL_H
#define EO_AS_IOCTL_H

#include <linux/types.h>   /* For u32, u64, etc. */

/*
 * Example 'magic' number for our IOCTL commands
 */
#define EO_AS_IOC_MAGIC  0xB0

/*
 * Convert old Windows IOCTL_* definitions to Linux _IO, _IOR, _IOW, _IOWR
 * and rename fields to Linux style (e.g. "dma_desc_count").
 */

/* Registry-like operation for reading/writing a register */
struct eo_as_registry_params {
    __u8  bar;       /* which BAR? */
    __u64 address;   /* register offset */
    __u32 value;     /* read/write value */
} __attribute__((packed));

#define EO_AS_IOC_SET_DMA_REG  _IOW (EO_AS_IOC_MAGIC, 0x01, struct eo_as_registry_params)
#define EO_AS_IOC_GET_DMA_REG  _IOWR(EO_AS_IOC_MAGIC, 0x02, struct eo_as_registry_params)

/* Example: get some DMA status as a single 32-bit value */
#define EO_AS_IOC_GET_DMA_STATUS _IOR(EO_AS_IOC_MAGIC, 0x03, __u32)

/*
 * Replacing GLOBAL_DATA_DMA_PARAMETERS with Linux naming
 */
struct eo_as_dma_params {
    __u32 dma_desc_count;     /* was DmaDescriptorsMaxCount */
    __u32 dma_buf_size;       /* was DmaDescriptorMaxBufferSize */
    __u32 dma_chan_count;     /* was DmaChannelsMaxCount */
} __attribute__((packed));

#define EO_AS_IOC_DMA_PARAMS_GET \
    _IOR(EO_AS_IOC_MAGIC, 0x04, struct eo_as_dma_params)

/**
 * @brief Number of DMA channels.
 */
#define MAX_NUM_CHANNELS       8

/**
 * @brief Number of DMA descriptors per channel.
 */
#define MAX_NUM_DESCRIPTORS    6

struct eo_as_dma_desc {
    __u64 buf_va;
    __u64 buf_pa;
} __attribute__((packed));

struct eo_as_dma_channel {
    struct eo_as_dma_desc descs[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

struct eo_as_mem_map {
    struct eo_as_dma_channel chans[MAX_NUM_CHANNELS];
} __attribute__((packed));

#define EO_AS_IOC_MEM_MAP_GET \
    _IOR(EO_AS_IOC_MAGIC, 0x05, struct eo_as_mem_map)

/*
 * If you previously had event-handle definitions, rename them likewise.
 * In Linux, there's no direct concept of "HANDLE", so you might store a user token:
 */
struct eo_as_event_desc {
    __u64 event_handle;  /* user-provided token */
} __attribute__((packed));

struct eo_as_event_chan {
    struct eo_as_event_desc events[MAX_NUM_DESCRIPTORS];
} __attribute__((packed));

struct eo_as_event_data {
    struct eo_as_event_chan chans[MAX_NUM_CHANNELS];
} __attribute__((packed));

#define EO_AS_IOC_EVENT_HANDLE_SET \
    _IOW(EO_AS_IOC_MAGIC, 0x06, struct eo_as_event_data)
#define EO_AS_IOC_EVENT_HANDLE_GET \
    _IOR(EO_AS_IOC_MAGIC, 0x07, struct eo_as_event_data)

/*
 * The user can now call these commands with ioctl(fd, ..., &argStruct).
 */

/**
 * @brief Define DMA descriptors region addresses.
 *
 * These definitions specify the register addresses within the DMA descriptors region,
 * including control registers, descriptor count, and indexing registers.
 */
#define DEVICE_GLOBAL_DRV_VER                    0xb     ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ENABLE      0x0003  ///< Global enable Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS      0x0005  ///< Global status Interrupt register (FIFO occupancy)
#define DEVICE_GLOBAL_INTERRUPT_FPGA_ACK         0x0004  ///< Global status Interrupt register
#define DEVICE_GLOBAL_INTERRUPT_FPGA_DATA        0x0006  ///< Global Interrupt register data (read before ACK)
#define DEVICE_GLOBAL_DMA_ENABLE_FPGA_DATA       0x0008  ///< Global DMA enable register data
#define DEVICE_GLOBAL_DMA_MAX_DMA_CHANELS_DATA   0x0009  ///< Global DMA max channels register data
#define DEVICE_GLOBAL_DMA_PPS_TRIGER			 0x0017  ///< Global PPS triger bit 0: Enable, bit 1: Rising/Fall

///<DMA control register address
#define DEVICE_GLOBAL_DMA_REG_CONTROL              0x0100  ///< Register address for DMA control operations.

///<DMA descriptors number register address
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_NUMBER   0x0101  ///< Register address for querying the number of DMA descriptors.

///<DMA get descriptor index register address
#define DEVICE_GLOBAL_DMA_REG_GET_DESCRIPTOR_INDEX 0x0102  ///< Register address for obtaining the current descriptor index.

///<DMA descriptors table base address
#define DEVICE_GLOBAL_DMA_REG_DESCRIPTORS_TABLE    0x0800  ///< Base address of the DMA descriptors table.

/**
 * transform_fpga_address - Transform the given FPGA address
 * @address: The original FPGA address to be transformed
 *
 * This function shifts the input address left by 2 bits, effectively
 * multiplying by 4. Typically used to map the FPGA's address space
 * according to specific hardware requirements.
 *
 * Return: The transformed (shifted) FPGA address.
 */
static inline u64 transform_fpga_address(u64 address)
{
	return address << 2; /* Multiply by 4 */
}
#endif /* EO_AS_IOCTL_H */
