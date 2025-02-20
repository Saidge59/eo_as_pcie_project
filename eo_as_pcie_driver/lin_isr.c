/**
 * @file lin_isr.c
 * @brief Linux-style interrupt handling and deferred processing (bottom-half).
 *
 * Replaces Windows-based logic from IsrDPC.c. We define:
 * 1) A top-half IRQ handler (`eo_as_irq_handler`).
 * 2) A bottom-half function (`eo_as_irq_bottom_half`) using a tasklet.
 * 3) Routines to enable/disable FPGA interrupts if needed.
 *
 * (C) 2013-2023 Aplit-Soft LTD & El-Or Systems LTD. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/interrupt.h> /* request_irq, free_irq, tasklet_schedule, etc. */
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/errno.h>
#include "hal_hwlayer.h"    /* For hal_is_our_interrupt, hal_read_interrupt_data, hal_global_interrupt_ack,
                               hal_transform_fpga_address, and DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS if they live here */
#include "lin_isr.h"        /* Declarations for eo_as_irq_handler, etc. */
#include "eo_as_device.h"   /* Definition of struct eo_as_device: includes dev->hal, dev->lock, etc. */

/*
 * Example 'struct eo_as_device' (from eo_as_device.h):
 * ----------------------------------------------------
 * struct eo_as_device {
 *     bool device_on;
 *     spinlock_t lock;
 *     struct hal_context hal;
 *     // arrays for DMA channels, etc.
 * };
 */

/* 
 * We'll define a static tasklet struct here. We'll init it in the IRQ top-half each time 
 * or during module/device init. We'll store the device pointer in ->data.
 */
static struct tasklet_struct eo_as_tasklet;

/*
 * Bottom-half function (was EvtDPC in Windows).
 * Called when the tasklet runs, receiving 'data' as an unsigned long.
 */
static void eo_as_irq_bottom_half(unsigned long data)
{
    struct eo_as_device *dev = (struct eo_as_device *)data;
    u32 regValue;
    u32 interruptFifoValue;
    u16 descriptor, channel;

    pr_debug("lin_isr: bottom_half started\n");

    /* Acquire spinlock if modifying shared data in dev */
    spin_lock(&dev->lock);

    do {
        /* Read interrupt data from HAL */
        regValue = hal_read_interrupt_data(&dev->hal);

        /* Example decoding high/low 16 bits: 
         * high -> descriptor, low -> channel
         */
        descriptor = (u16)((regValue & 0xFFFF0000) >> 16);
        channel    = (u16)(regValue & 0x0000FFFF);

        pr_debug("lin_isr: channel=%u, descriptor=%u, regValue=0x%x\n",
                 channel, descriptor, regValue);

        /* If needed, handle completion events, e.g. 
         * if (channel < dev->dma_chan_count && descriptor < dev->dma_desc_count) {
         *     complete(&dev->dma_done[channel][descriptor]);
         * }
         */

        /* Read interrupt FIFO to see if more interrupts remain */
        interruptFifoValue = hal_mmio_bar_read32(
            &dev->hal, 0,
            hal_transform_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS)
        );

        pr_debug("lin_isr: interruptFifoValue=%u after reading regValue\n", interruptFifoValue);

    } while (interruptFifoValue != 0);

    /* Acknowledge in hardware */
    hal_global_interrupt_ack(&dev->hal);

    spin_unlock(&dev->lock);

    pr_debug("lin_isr: bottom_half done\n");
}

/*
 * The top-half IRQ handler (replaces EvtISR). 
 */
irqreturn_t eo_as_irq_handler(int irq, void *dev_id)
{
    struct eo_as_device *dev = dev_id;

    pr_debug("lin_isr: in top-half\n");

    /* If device_on == false, ignore. Or if not our interrupt, ignore. */
    if (!dev->device_on)
        return IRQ_NONE;

    /* Check if it belongs to our device */
    if (hal_is_our_interrupt(&dev->hal) != HAL_SUCCESS) {
        pr_debug("lin_isr: not our interrupt\n");
        return IRQ_NONE;
    }

    /* 
     * We'll init the tasklet each time or do it once at driver init. 
     * Here we do it each time for demonstration.
     *
     * tasklet_init(struct tasklet_struct *t, void (*func)(unsigned long), unsigned long data);
     */
    tasklet_init(&eo_as_tasklet, eo_as_irq_bottom_half, (unsigned long)dev);

    /* schedule bottom half */
    tasklet_schedule(&eo_as_tasklet);

    return IRQ_HANDLED;
}

/*
 * If you previously had EvtInterruptEnable/EvtInterruptDisable,
 * in Linux we typically do hal_interrupt_enable/disable in .probe/.remove
 * or power management routines. But if you want inline helpers:
 */
void eo_as_interrupt_enable(struct eo_as_device *dev)
{
    hal_interrupt_enable(&dev->hal);
    dev->device_on = true;
}

void eo_as_interrupt_disable(struct eo_as_device *dev)
{
    hal_interrupt_disable(&dev->hal);
    dev->device_on = false;
}

