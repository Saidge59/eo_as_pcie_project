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
#include <linux/interrupt.h> 
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/eventfd.h>
#include <linux/types.h>
#include <linux/errno.h>
#include "hal_hwlayer.h"
#include "lin_isr.h"
#include "eo_as_device.h"

/**
 * @brief Static tasklet structure for deferred interrupt processing.
 *
 * Initialized in the IRQ top-half each time or during module/device init.
 * Stores the device pointer in ->data.
 */
static struct tasklet_struct eo_as_tasklet;

static void eo_as_irq_bottom_half(unsigned long data)
{
    struct eo_as_device *dev = (struct eo_as_device *)data;
    u32 regValue;
    u32 interruptFifoValue;
    u16 descriptor, channel;
    u64 event_fd;
    struct eventfd_ctx *event_ctx;

    pr_info("lin_isr: bottom_half started\n");

    spin_lock(&dev->lock);

    do
    {
        regValue = hal_read_interrupt_data(&dev->hal);

        descriptor = (u16)((regValue & 0xFFFF0000) >> 16);
        channel = (u16)(regValue & 0x0000FFFF);

        pr_info("lin_isr: channel=%u, descriptor=%u, regValue=0x%x\n",
                channel, descriptor, regValue);

        if (channel < MAX_NUM_CHANNELS && descriptor < MAX_NUM_DESCRIPTORS)
        {
            event_fd = dev->event_data.chans[channel].events[descriptor].event_handle;
            event_ctx = dev->event_data.chans[channel].events[descriptor].event_ctx;

            if (event_fd > 0 && event_ctx)
            {
                eventfd_signal(event_ctx, 1);
            }
            else
            {
                pr_err("Failed to get eventfd or event_context for channel=%u, descriptor=%u\n", channel, descriptor);
            }
        }

        interruptFifoValue = hal_mmio_bar_read32(
            &dev->hal, 0,
            trans_form_fpga_address(DEVICE_GLOBAL_INTERRUPT_FPGA_STATUS));

        pr_info("lin_isr: interruptFifoValue=%u after reading regValue\n", interruptFifoValue);

    } while (interruptFifoValue != 0);

    hal_global_interrupt_ack(&dev->hal);

    spin_unlock(&dev->lock);

    pr_debug("lin_isr: bottom_half done\n");
}

irqreturn_t eo_as_irq_handler(int irq, void *dev_id)
{
    struct eo_as_device *dev = dev_id;

    pr_info("lin_isr: in top-half\n");

    if (!dev->device_on)
        return IRQ_NONE;
    pr_info("lin_isr: check if is our interrupt\n");

    if (hal_is_our_interrupt(&dev->hal) != HAL_SUCCESS)
    {
        pr_info("lin_isr: not our interrupt\n");
        return IRQ_NONE;
    }
    pr_info("lin_isr: is our interrupt\n");

    tasklet_init(&eo_as_tasklet, eo_as_irq_bottom_half, (unsigned long)dev);

    tasklet_schedule(&eo_as_tasklet);

    return IRQ_HANDLED;
}

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