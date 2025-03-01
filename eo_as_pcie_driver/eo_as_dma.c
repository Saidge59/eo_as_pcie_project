/**
 * @file eo_as_dma.c
 * @brief DMA memory allocation and free functions for the EO_AS FPGA driver.
 *
 * This file implements functions to allocate and free DMA-coherent buffers for the
 * FPGA driver. The functions use the definitions provided in eo_as_device.h.
 *
 * The total memory allocated will be:
 *   MAX_NUM_CHANNELS x MAX_NUM_DESCRIPTORS x DESCRIPTOR_BUF_SIZE
 * For example, with 8 channels and 8 descriptors per channel (each 64 MB),
 * the total allocated memory is 8 x 8 x 64 MB = 4096 MB (or 4 GB).
 *
 * Copyright © 2025 Aplit-Soft LTD & El-Or Systems LTD.
 * All Rights Reserved.
 *
 * THIS SOFTWARE is proprietary and confidential. Duplication or disclosure without explicit
 * written permission is prohibited.
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/printk.h>
#include "eo_as_device.h"

int eo_as_allocate_dma_buffers(struct eo_as_device *eadev)
{
    struct dma_device_context *ctx = eadev->dma_ctx;
    int ch, desc;
    for (ch = 0; ch < MAX_NUM_CHANNELS; ch++)
    {
        for (desc = 0; desc < MAX_NUM_DESCRIPTORS; desc++)
        {
            ctx->channels[ch].descriptors[desc].buffer =
                dma_alloc_coherent(ctx->dev,
                                   DESCRIPTOR_BUF_SIZE,
                                   &ctx->channels[ch].descriptors[desc].dma_handle,
                                   GFP_KERNEL);
            if (!ctx->channels[ch].descriptors[desc].buffer)
            {
                dev_err(ctx->dev,
                        "Failed to allocate DMA buffer for channel %d, descriptor %d\n",
                        ch, desc);
            }
            ctx->channels[ch].descriptors[desc].ev_up = NULL;

            eadev->mem_map.chans[ch].descs[desc].buf_va = (uint64_t)ctx->channels[ch].descriptors[desc].buffer;
            eadev->mem_map.chans[ch].descs[desc].buf_pa = (uint64_t)ctx->channels[ch].descriptors[desc].dma_handle;

            dev_info(ctx->dev,
                "Allocated DMA buffer for channel %d, descriptor %d: VA=%p, DMA=%pad\n",
                ch, desc,
                ctx->channels[ch].descriptors[desc].buffer,
                &ctx->channels[ch].descriptors[desc].dma_handle);
        }
    }
    return 0;
}
EXPORT_SYMBOL(eo_as_allocate_dma_buffers);

void eo_as_free_dma_buffers(struct eo_as_device *eadev)
{
    struct dma_device_context *ctx = eadev->dma_ctx;
    int ch, desc;
    for (ch = 0; ch < MAX_NUM_CHANNELS; ch++)
    {
        for (desc = 0; desc < MAX_NUM_DESCRIPTORS; desc++)
        {
            if (ctx->channels[ch].descriptors[desc].buffer)
            {
                dma_free_coherent(ctx->dev, DESCRIPTOR_BUF_SIZE,
                                  ctx->channels[ch].descriptors[desc].buffer,
                                  ctx->channels[ch].descriptors[desc].dma_handle);
                ctx->channels[ch].descriptors[desc].buffer = NULL;
                eadev->mem_map.chans[ch].descs[desc].buf_pa = 0;
                eadev->mem_map.chans[ch].descs[desc].buf_va = 0;
            }
        }
    }
}
EXPORT_SYMBOL(eo_as_free_dma_buffers);
