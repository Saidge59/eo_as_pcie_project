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

#include <linux/types.h>
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
 * @brief Structure for hardware interrupt data.
 *
 * Contains information about the interrupt, including device ID, IRQ line, and vector.
 */
struct hal_interrupt_data {
    void   *dev_id;
    unsigned int irq_line;
    unsigned int vector;
};

/**
 * @brief Structure for hardware abstraction layer (HAL) context.
 *
 * Holds the parent context, memory-mapped BARs, interrupt data, and synchronization lock.
 */
struct hal_context {
    void *parent_context;

    void __iomem *bar[DRV_MAX_NUM_BARS];
    size_t        bar_length[DRV_MAX_NUM_BARS];
    size_t        num_bars;

    struct hal_interrupt_data intr_data;

    spinlock_t dma_data_lock;
};

/**
 * @brief Enumeration for HAL operation status codes.
 *
 * Defines possible return values for HAL functions indicating success or various errors.
 */
enum hal_status {
    HAL_SUCCESS = 0,
    HAL_FAILED,
    HAL_NULL_MEMORY_ERROR,
    HAL_MEMORY_ERROR,
    HAL_PCIE_ERROR,
};

/**
 * @brief Writes a 32-bit value to a memory-mapped BAR register.
 *
 * @param hctx Pointer to the HAL context.
 * @param bar Index of the BAR to write to.
 * @param offset Offset within the BAR.
 * @param value 32-bit value to write.
 */
void hal_mmio_bar_write32(struct hal_context *hctx, u8 bar, u64 offset, u32 value);

/**
 * @brief Reads a 32-bit value from a memory-mapped BAR register.
 *
 * @param hctx Pointer to the HAL context.
 * @param bar Index of the BAR to read from.
 * @param offset Offset within the BAR.
 * @return The 32-bit value read from the register.
 */
u32  hal_mmio_bar_read32(struct hal_context *hctx, u8 bar, u64 offset);

/**
 * @brief Disables interrupts for the device.
 *
 * @param hctx Pointer to the HAL context.
 */
void hal_interrupt_disable(struct hal_context *hctx);

/**
 * @brief Enables interrupts for the device.
 *
 * @param hctx Pointer to the HAL context.
 */
void hal_interrupt_enable(struct hal_context *hctx);

/**
 * @brief Checks if an interrupt belongs to this device.
 *
 * @param hctx Pointer to the HAL context.
 * @return HAL status indicating whether the interrupt is ours.
 */
enum hal_status hal_is_our_interrupt(struct hal_context *hctx);

/**
 * @brief Reads interrupt data from the device.
 *
 * @param hctx Pointer to the HAL context.
 * @return 32-bit interrupt data value.
 */
u32 hal_read_interrupt_data(struct hal_context *hctx);

/**
 * @brief Checks if an interrupt is related to a specific DMA index.
 *
 * @param hctx Pointer to the HAL context.
 * @param dma_index Index of the DMA channel to check.
 * @return HAL status indicating whether the interrupt is DMA-related.
 */
enum hal_status hal_is_dma_interrupt(struct hal_context *hctx, u32 dma_index);

/**
 * @brief Acknowledges a global interrupt.
 *
 * @param hctx Pointer to the HAL context.
 */
void hal_global_interrupt_ack(struct hal_context *hctx);

/**
 * @brief Aligns an offset to a multiple of 4.
 *
 * Adjusts the offset to the next multiple of 4 if it is not already aligned.
 *
 * @param offset The offset to align.
 * @return The aligned offset.
 */
static inline u64 hal_align_to_multiple_of4(u64 offset)
{
	if (offset % 4 != 0)
		offset += (4 - (offset % 4));
	return offset;
}

/**
 * @brief Reads a 32-bit register value from a memory-mapped base address.
 *
 * Aligns the offset to a multiple of 4 before reading.
 *
 * @param base Base address of the memory-mapped region.
 * @param offset Offset within the base address.
 * @return The 32-bit value read from the register.
 */
static inline u32 hal_read_register(void __iomem *base, u64 offset)
{
	u64 aligned = hal_align_to_multiple_of4(offset);
	return ioread32((void __iomem *)((u8 __iomem *)base + aligned));
}

/**
 * @brief Writes a 32-bit value to a register at a memory-mapped base address.
 *
 * Aligns the offset to a multiple of 4 before writing.
 *
 * @param base Base address of the memory-mapped region.
 * @param offset Offset within the base address.
 * @param value 32-bit value to write.
 */
static inline void hal_write_register(void __iomem *base, u64 offset, u32 value)
{
	u64 aligned = hal_align_to_multiple_of4(offset);
	iowrite32(value, (void __iomem *)((u8 __iomem *)base + aligned));
}

#endif /* _HAL_HWLAYER_H_ */
