/**
 * @file hal_hwlayer.h
 * @brief Hardware Abstraction Layer (HAL) for the Linux kernel driver.
 *
 * Provides hardware-specific structures, definitions, and function declarations
 * for interacting with memory-mapped I/O registers and interrupts on the FPGA device.
 *
 * Re-implemented from the KMDF-based HwLayer.h for the Linux environment.
 */

#ifndef _HAL_HWLAYER_H_
#define _HAL_HWLAYER_H_

#include <linux/types.h>  /* for u64 */
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

/* 
 * Driver version and constants 
 */
#define DR_VER            0xAEAE0104
#define DRV_MAX_NUM_BARS  2
#define BAR0              0
#define BUFFERS_COUNT     256
#define MAX_DMA_COUNT_8   8
#define MAX_DMA_COUNT_16  16
#define DMA_REG_OFFSET    0x100

/**
 * @struct hal_interrupt_data
 * @brief Stores interrupt-related data in Linux style.
 */
struct hal_interrupt_data {
    void   *dev_id;      /* Pointer back to device context */
    unsigned int irq_line;
    unsigned int vector;
};

/**
 * @struct hal_context
 * @brief HAL context for the device.
 */
struct hal_context {
    /* Points to parent device (e.g., struct pci_dev or your driver’s context). */
    void *parent_context;

    /* Memory-mapped I/O regions. */
    void __iomem *bar[DRV_MAX_NUM_BARS];
    size_t        bar_length[DRV_MAX_NUM_BARS];
    size_t        num_bars;

    /* Interrupt information. */
    struct hal_interrupt_data intr_data;

    /* Spinlock for protecting shared data, if needed. */
    spinlock_t dma_data_lock;
};

/**
 * @enum hal_status
 * @brief Status codes returned by HAL functions.
 */
enum hal_status {
    HAL_SUCCESS = 0,
    HAL_FAILED,
    HAL_NULL_MEMORY_ERROR,
    HAL_MEMORY_ERROR,
    HAL_PCIE_ERROR,
};

/* Function declarations for read/write registers */
void hal_mmio_bar_write32(struct hal_context *hctx, u8 bar, u64 offset, u32 value);
u32  hal_mmio_bar_read32(struct hal_context *hctx, u8 bar, u64 offset);

/* Interrupt-related functions */
void hal_interrupt_disable(struct hal_context *hctx);
void hal_interrupt_enable(struct hal_context *hctx);
enum hal_status hal_is_our_interrupt(struct hal_context *hctx);
u32 hal_read_interrupt_data(struct hal_context *hctx);
enum hal_status hal_is_dma_interrupt(struct hal_context *hctx, u32 dma_index);
void hal_global_interrupt_ack(struct hal_context *hctx);


/**
 * @brief Transforms the provided FPGA address by left shifting by 2 (multiply by 4).
 *
 * @param address The original FPGA address.
 * @return The transformed FPGA address.
 */
static inline u64 hal_transform_fpga_address(u64 address)
{
	return address << 2;
}

/**
 * @brief Aligns the provided offset to the nearest multiple of 4.
 *
 * @param offset The original offset value.
 * @return The aligned offset value.
 */
static inline u64 hal_align_to_multiple_of4(u64 offset)
{
	if (offset % 4 != 0)
		offset += (4 - (offset % 4));
	return offset;
}

/**
 * @brief Reads a 32-bit value from a specified register in memory.
 *
 * @param base  Base address of the register set (must be valid).
 * @param offset Offset (in bytes) within that base.
 * @return The 32-bit value read from the register.
 */
static inline u32 hal_read_register(void __iomem *base, u64 offset)
{
	u64 aligned = hal_align_to_multiple_of4(offset);
	return ioread32((void __iomem *)((u8 __iomem *)base + aligned));
}

/**
 * @brief Writes a 32-bit value to a specified register in memory.
 *
 * @param base  Base address of the register set.
 * @param offset Offset (in bytes) within that base.
 * @param value The 32-bit value to write.
 */
static inline void hal_write_register(void __iomem *base, u64 offset, u32 value)
{
	u64 aligned = hal_align_to_multiple_of4(offset);
	iowrite32(value, (void __iomem *)((u8 __iomem *)base + aligned));
}
#endif /* _HAL_HWLAYER_H_ */
