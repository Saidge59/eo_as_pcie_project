#ifndef LIN_ISR_H
#define LIN_ISR_H

#include <linux/interrupt.h>
#include "public.h"

/* Forward declaration of your device struct. Adjust name if needed. */
struct eo_as_device;

/**
 * @brief The top-half IRQ handler. Replaces EvtISR in Windows.
 *
 * @param irq     The IRQ line number.
 * @param dev_id  Pointer to your struct eo_as_device.
 * @return IRQ_HANDLED if the interrupt is ours, else IRQ_NONE.
 */
irqreturn_t eo_as_irq_handler(int irq, void *dev_id);

/**
 * @brief Helper to enable interrupts in the FPGA (like EvtInterruptEnable).
 *
 * @param dev Pointer to your device struct.
 */
void eo_as_interrupt_enable(struct eo_as_device *dev);

/**
 * @brief Helper to disable interrupts in the FPGA (like EvtInterruptDisable).
 *
 * @param dev Pointer to your device struct.
 */
void eo_as_interrupt_disable(struct eo_as_device *dev);

#endif /* LIN_ISR_H */
