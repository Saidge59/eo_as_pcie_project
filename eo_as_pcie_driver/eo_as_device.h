/* eo_as_device.h */
#ifndef EO_AS_DEVICE_H
#define EO_AS_DEVICE_H

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include "hal_hwlayer.h"
#include "public.h"

/**
 * @brief DMA descriptor structure.
 *
 * This structure represents a single DMA descriptor that holds a DMA-coherent memory
 * buffer along with its corresponding DMA handle. It also contains a pointer to an
 * eventfd context used for per-descriptor notifications (kernel→user signaling).
 */
struct dma_descriptor {
    void               *buffer;      /**< Kernel virtual address of the allocated DMA buffer */
    dma_addr_t          dma_handle;  /**< DMA (bus) address of the allocated buffer */
    struct eventfd_ctx *ev_up;       /**< Eventfd context for signaling completion (set via ioctl) */
};

/**
 * @brief DMA channel structure.
 *
 * A DMA channel consists of an array of DMA descriptors. Each channel can process multiple
 * DMA operations concurrently using its descriptors.
 */
struct dma_channel {
    struct dma_descriptor descriptors[MAX_NUM_DESCRIPTORS]; /**< Array of DMA descriptors for the channel */
};

/**
 * @brief DMA device context.
 *
 * This context contains the device pointer and an array of DMA channels. It is used by the
 * DMA driver to manage DMA buffer allocation and per-descriptor event signaling.
 */
struct dma_device_context {
    struct device    *dev;         /**< Pointer to the associated device structure (set during probe) */
    int               num_channels; /**< Number of DMA channels (expected to be NUM_CHANNELS) */
    struct dma_channel *channels;   /**< Pointer to an allocated array of DMA channels */
};

/* 
 * struct eo_as_device: The device context for our FPGA device.
 */
struct eo_as_device {

        

    struct eo_as_dma_params dma_params;  /* if you store the DMA info here */
    struct eo_as_mem_map mem_map;        /* if you store mem_map here */
    struct eo_as_event_data event_data;  /* if you store event data here */
    struct dma_device_context *dma_ctx;   /*This context contains the device pointer and an array of DMA channels  */

    bool device_on;        /* example */
  

    /* anything else needed by your driver */
    /* BAR mappings */
    void __iomem       *bar0;       /**< Mapped address for BAR0 */
    void __iomem       *bar1;       /**< Mapped address for BAR1 */
    size_t bar0_len;
    struct pci_dev     *pdev;       /**< PCI device pointer */
    spinlock_t          lock;       /**< Lock for device operations */
  
    struct hal_context  hal;        /**< HAL context for register access */
    bool                irq_registered; /**< Indicates if IRQ is registered */
    int                 irq_line;   /**< IRQ line number */
    dev_t               devt;       /**< Device number for the char device */
    struct cdev         cdev;       /**< Character device structure */
    struct class       *class;      /**< Device class pointer for /dev node creation */
    struct device      *dev;        /**< Pointer to the created device node */
};

#endif /* EO_AS_DEVICE_H */
