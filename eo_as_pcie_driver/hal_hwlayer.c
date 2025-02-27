/**
 * @file hal_hwlayer.c
 * @brief Implementation of the HAL for the Linux kernel driver.
 *
 * Re-implements the read/write register, interrupt checks, etc., from KMDF style
 * to Linux kernel style.
 */

#include <linux/io.h>      // for iowrite32, ioread32
#include <linux/printk.h>  // for pr_info, etc.
#include "hal_hwlayer.h"



u32 hal_mmio_bar_read32(struct hal_context *hctx, u8 bar, u64 offset)
{
    u32 data = 0;
    if (!hctx || bar >= hctx->num_bars || bar >= DRV_MAX_NUM_BARS){
        pr_debug("hal_hwlayer: Read32 from failure bar =%d offset=0x%llx \n",
        bar, offset);
        return 0xFFFFFFFF;
    }
      

    if (offset < hctx->bar_length[bar] && hctx->bar[bar]) {
        data = ioread32((void __iomem *)(hctx->bar[bar] + offset));
        pr_debug("hal_hwlayer: Read32 from bar=%d offset=0x%llx => 0x%x\n",
                 bar, offset, data);
    }
    return data;
}

void hal_mmio_bar_write32(struct hal_context *hctx, u8 bar, u64 offset, u32 value)
{
    if (!hctx || bar >= hctx->num_bars || bar >= DRV_MAX_NUM_BARS)
        return;

    if (offset < hctx->bar_length[bar] && hctx->bar[bar]) {
        u64 aligned = hal_align_to_multiple_of4(offset);
        iowrite32(value, (void __iomem *)(hctx->bar[bar] + aligned));
        pr_info("hal_hwlayer: Write32 to bar=%d offset=0x%llx val=0x%x\n",
                bar, aligned, value);
    }
}

void hal_interrupt_disable(struct hal_context *hctx)
{
    /* Example: Write 0 to a global interrupt enable register at offset 0x200 */
    pr_info("hal_hwlayer: Disabling interrupts\n");
    hal_mmio_bar_write32(hctx, BAR0, 0xc, 0x0);
}

void hal_interrupt_enable(struct hal_context *hctx)
{
    /* Example: Write 1 to the same global interrupt enable register. */
    pr_info("hal_hwlayer: Enabling interrupts\n");
    hal_mmio_bar_write32(hctx, BAR0, 0xc, 0x1);
}

enum hal_status hal_is_our_interrupt(struct hal_context *hctx)
{
    u32 reg_val;
    if (!hctx) 
        return HAL_FAILED;
    /* Example: read a global interrupt status reg at offset 0x104 */
    reg_val = hal_mmio_bar_read32(hctx, BAR0, 0x14);
    pr_info("hal_hwlayer: Checking global interrupt status=0x%x\n", reg_val);

    if (reg_val != 0)
        return HAL_SUCCESS;
    return HAL_FAILED;
}

u32 hal_read_interrupt_data(struct hal_context *hctx)
{
    if (!hctx)
        return 0xFFFFFFFF;
    /* e.g., read an interrupt data reg at offset 0x108 */
    return hal_mmio_bar_read32(hctx, BAR0, 0x18);
}

enum hal_status hal_is_dma_interrupt(struct hal_context *hctx, u32 dma_index)
{
    if (!hctx || dma_index > MAX_DMA_COUNT_16)
        return HAL_FAILED;
    /* Example logic. If needed, read a DMA interrupt status reg. */
    return HAL_SUCCESS;
}

void hal_global_interrupt_ack(struct hal_context *hctx)
{
    /* e.g., write 1 to ack register */
    pr_info("hal_hwlayer: Global interrupt Ack\n");
    hal_mmio_bar_write32(hctx, BAR0, 0x10, 0x1);
}
