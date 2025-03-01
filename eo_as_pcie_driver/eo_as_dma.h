/**
 * @file eo_as_dma.h
 * @brief DMA memory allocation function prototypes for the EO_AS FPGA driver.
 *
 * This header declares the functions for allocating and freeing DMA-coherent buffers
 * for each DMA descriptor in the EO_AS driver. The allocation is done per channel
 * and per descriptor, as defined in the device context in eo_as_device.h.
 *
 * The total memory allocated will be:
 *   MAX_NUM_CHANNELS x MAX_NUM_DESCRIPTORS x DESCRIPTOR_BUF_SIZE
 * For example, with 8 channels and 8 descriptors per channel (each 64 MB),
 * the total allocated memory is 8 x 8 x 64 MB = 4096 MB (4 GB).
 *
 * Copyright © 2025 Aplit-Soft LTD & El-Or Systems LTD.
 * All Rights Reserved.
 *
 * THIS SOFTWARE is proprietary and confidential. Duplication or disclosure without explicit
 * written permission is prohibited.
 */

#ifndef EO_AS_DMA_H
#define EO_AS_DMA_H

#include <linux/device.h>
#include "eo_as_device.h"
#include "public.h"

/**
 * @brief Allocates DMA-coherent buffers for each descriptor in every DMA channel.
 *
 * This function iterates over each DMA channel and each descriptor, allocating a DMA-coherent
 * memory buffer of size DESCRIPTOR_BUF_SIZE for each descriptor. If an allocation fails, the function
 * frees all previously allocated buffers and returns an error code.
 *
 * @param eadev Pointer to the device context for our FPGA device.
 *
 * @return 0 on success, or -ENOMEM if any allocation fails.
 */
int eo_as_allocate_dma_buffers(struct eo_as_device *eadev);

/**
 * @brief Frees all DMA-coherent buffers allocated for each descriptor.
 *
 * This function iterates over each DMA channel and descriptor, freeing any allocated DMA-coherent
 * buffer (allocated via eo_as_allocate_dma_buffers()) and resets the buffer pointer to NULL.
 *
 * @param eadev Pointer to the device context for our FPGA device.
 */
void eo_as_free_dma_buffers(struct eo_as_device *eadev);

#endif /* EO_AS_DMA_H */
