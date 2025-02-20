/**
 * @file eo_as_pci_driver.c
 * @brief A full Linux kernel driver for a PCI-based FPGA device.
 *
 * This driver demonstrates how to:
 * 1. Detect a PCI device (Xilinx FPGA).
 * 2. Map BARs (memory-mapped registers).
 * 3. Handle an interrupt line.
 * 4. Provide simple read/write register access.
 * 5. Expose a char device for user-space open/release/read/write operations.
 *
 * Copyright © 2023 Aplit-Soft LTD & El-Or Systems LTD.
 * All Rights Reserved.
 *
 * THIS SOFTWARE is proprietary and confidential. Duplication or disclosure
 * without explicit written permission is prohibited.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>

#include <linux/uaccess.h> /* For copy_to_user, copy_from_user */
#include "eo_as_ioctl.h" /*IOCTL function definition*/
#include "eo_as_device.h"   /* Definitions structs */
#include "eo_as_dma.h"      /*DMA function definition*/
#include "hal_hwlayer.h" /* Suppose you implement your HAL read/write calls here */


/* Example PCI device IDs (Xilinx vendor 0x10EE, device 0x9011) */
#define EO_AS_VENDOR_ID  0x10EE
#define EO_AS_DEVICE_ID  0x9011

/* Driver name and a unique major number for the char device */
#define EO_AS_DRIVER_NAME  "eo_as_fpga_driver"
static int eo_as_driver_major = 270;  /* Example major number, adjust if needed */

/* Forward declarations. */
static int eo_as_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void eo_as_pci_remove(struct pci_dev *pdev);
static int eo_as_dev_open(struct inode *inode, struct file *filp);
static int eo_as_dev_release(struct inode *inode, struct file *filp);
static ssize_t eo_as_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static ssize_t eo_as_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos);
static irqreturn_t eo_as_fpga_irq_handler(int irq, void *dev_id);

/* PCI device ID table */
static const struct pci_device_id eo_as_pci_ids[] = {
    { PCI_DEVICE(EO_AS_VENDOR_ID, EO_AS_DEVICE_ID) },
    { 0, } /* terminates list */
};
MODULE_DEVICE_TABLE(pci, eo_as_pci_ids);



/* PCI driver struct */
static struct pci_driver eo_as_pci_driver = {
    .name     = EO_AS_DRIVER_NAME,
    .id_table = eo_as_pci_ids,
    .probe    = eo_as_pci_probe,
    .remove   = eo_as_pci_remove,
};

/* File operations for our char device. */
static const struct file_operations eo_as_dev_fops = {
    .owner   = THIS_MODULE,
    .open    = eo_as_dev_open,
    .release = eo_as_dev_release,
    .read    = eo_as_dev_read,
    .write   = eo_as_dev_write,
    .unlocked_ioctl = eo_as_dev_ioctl,
};

/* 
 * Interrupt handler. Called when the FPGA triggers an interrupt.
 * We'll check if it's "our" interrupt, then read/ack as needed.
 */
static irqreturn_t eo_as_fpga_irq_handler(int irq, void *dev_id)
{
    struct eo_as_device *eadev = dev_id;
    /* Example: hal_is_our_interrupt checks some FPGA register. */
    if (hal_is_our_interrupt(&eadev->hal) == HAL_SUCCESS) {
        /* read data or do ack */
        u32 data = hal_read_interrupt_data(&eadev->hal);
        pr_info("eo_as_fpga_driver: IRQ data=0x%x\n", data);

        hal_global_interrupt_ack(&eadev->hal); /* ack interrupt in FPGA */
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}

/**
 * @brief Probe routine for the PCI driver.
 *
 * Called when the PCI core finds a device matching our device ID table.
 * The routine enables the device, maps BARs, requests IRQs, initializes the HAL,
 * allocates DMA buffers, and registers a char device.
 *
 * @param pdev Pointer to the PCI device.
 * @param ent Pointer to the matching PCI device ID entry.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int eo_as_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct eo_as_device *eadev;
    int ret;
    resource_size_t bar_start0, bar_len0;
    resource_size_t bar_start1, bar_len1;
    dev_t devno;
    int vector;

    pr_info("eo_as_fpga_driver: probe called for vendor=0x%x, device=0x%x\n",
            pdev->vendor, pdev->device);

    /* 1) Enable the PCI device */
    ret = pci_enable_device(pdev);
    if (ret) {
        pr_err("eo_as_fpga_driver: pci_enable_device failed: %d\n", ret);
        return ret;
    }

    /* 2) Allocate driver context */
    eadev = kzalloc(sizeof(*eadev), GFP_KERNEL);
    if (!eadev) {
        ret = -ENOMEM;
        goto err_disable;
    }
    eadev->pdev = pdev;
    pci_set_drvdata(pdev, eadev);

    spin_lock_init(&eadev->lock);

    /* 3) Enable bus mastering, needed for MSI or DMA on many PCIe devices */
    pci_set_master(pdev);

    /* 4) Request regions for all BARs; typically requests 0..5 if they exist */
    ret = pci_request_regions(pdev, EO_AS_DRIVER_NAME);
    if (ret) {
        pr_err("eo_as_fpga_driver: pci_request_regions failed: %d\n", ret);
        goto err_free;
    }

    /* 5) Read/Check BAR0 */
    bar_start0 = pci_resource_start(pdev, 0);
    bar_len0   = pci_resource_len(pdev, 0);
    if (!bar_start0 || !bar_len0) {
        pr_err("eo_as_fpga_driver: invalid BAR0\n");
        ret = -ENODEV;
        goto err_release;
    }

    /* 6) Read/Check BAR1 (or BAR2 if your device uses that) */
    bar_start1 = pci_resource_start(pdev, 2);  /* or index 2 if your device enumerates it there */
    bar_len1   = pci_resource_len(pdev, 2);
    if (!bar_start1 || !bar_len1) {
       pr_err("eo_as_fpga_driver: invalid BAR1\n");
        ret = -ENODEV;
        goto err_release;
    }

    /* 7) ioremap both BARs */
    eadev->bar0 = pci_iomap(pdev, 0, 0);
    if (!eadev->bar0) {
        pr_err("eo_as_fpga_driver: pci_iomap bar0 fail\n");
        ret = -ENOMEM;
        goto err_release;
    }
    eadev->bar1 = pci_iomap(pdev, 2, 0); /* or index 2 if needed */
    if (!eadev->bar1) {
       pr_err("eo_as_fpga_driver: pci_iomap bar1 fail\n");
        ret = -ENOMEM;
        goto err_unmap0;
    }

    /* Setup the HAL context for read/write, etc. */
    eadev->hal.bar[0]         = eadev->bar0;
    eadev->hal.bar_length[0]  = bar_len0;
    eadev->hal.bar[1]         = eadev->bar1;
    eadev->hal.bar_length[1]  = bar_len1;
    eadev->hal.num_bars       = 1;

    /* Allocate MSI vector and request IRQ */
    ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_MSIX | PCI_IRQ_LEGACY);
    if (ret < 0) {
        pr_err("eo_as_fpga_driver: pci_alloc_irq_vectors failed: %d\n", ret);
        goto err_unmap1;
    }
    vector = pci_irq_vector(pdev, 0);
    ret = request_irq(vector, eo_as_fpga_irq_handler, 0, 
                      EO_AS_DRIVER_NAME, eadev);
    if (ret) {
        pr_err("eo_as_fpga_driver: request_irq (vector %d) failed: %d\n", vector, ret);
        goto err_vectors;
    }
    eadev->irq_registered = true;
    eadev->irq_line = vector;

    /* enable interrupts in FPGA */
    hal_interrupt_enable(&eadev->hal);

    /* Allocate and initialize DMA buffers for FPGA operations */
    eadev->dma_ctx = (struct dma_device_context*)kzalloc(sizeof(struct dma_device_context), GFP_KERNEL);
    if (!eadev->dma_ctx) {
        ret = -ENOMEM;
        goto err_irq;
    }
    eadev->dma_ctx->dev = &pdev->dev;
    eadev->dma_ctx->num_channels = MAX_NUM_CHANNELS;
    eadev->dma_ctx->channels = kzalloc(sizeof(struct dma_channel) * MAX_NUM_CHANNELS, GFP_KERNEL);
    if (!eadev->dma_ctx->channels) {
        ret = -ENOMEM;
        goto err_free_dma_ctx;
    }
    ret = eo_as_allocate_dma_buffers(eadev->dma_ctx);
    if (ret < 0)
        goto err_free_dma_channels;

    /* Register char device */
    ret = alloc_chrdev_region(&devno, 0, 1, EO_AS_DRIVER_NAME);
    if (ret < 0) {
        pr_err("eo_as_fpga_driver: alloc_chrdev_region failed: %d\n", ret);
        goto err_irq;
    }
    eadev->devt = devno;  /* store in context */

    cdev_init(&eadev->cdev, &eo_as_dev_fops);
    eadev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&eadev->cdev, eadev->devt, 1);
    if (ret) {
        pr_err("eo_as_fpga_driver: cdev_add failed: %d\n", ret);
        unregister_chrdev_region(eadev->devt, 1);
        goto err_irq;
    }
   
    /* Create a device node under /dev */
    eadev->class = class_create(THIS_MODULE, EO_AS_DRIVER_NAME);
    if (IS_ERR(eadev->class)) {
        ret = PTR_ERR(eadev->class);
        unregister_chrdev_region(eadev->devt, 1);
        goto err_free_dma_channels;
    }
    eadev->dev = device_create(eadev->class, NULL, eadev->devt, NULL, EO_AS_DRIVER_NAME);
    if (IS_ERR(eadev->dev)) {
        ret = PTR_ERR(eadev->dev);
        class_destroy(eadev->class);
        unregister_chrdev_region(eadev->devt, 1);
        goto err_free_dma_channels;
    }
    
    pr_info("eo_as_fpga_driver: probe success, major=%d minor=%d, bar0=0x%llx, bar1=0x%llx, MSI vector=%d\n",
            MAJOR(eadev->devt), MINOR(eadev->devt),
            (unsigned long long)bar_start0,
            (unsigned long long)bar_start1,
            vector);
    return 0;
err_free_dma_channels:
    eo_as_free_dma_buffers(eadev->dma_ctx);
    kfree(eadev->dma_ctx->channels);
err_free_dma_ctx:
    kfree(eadev->dma_ctx);
err_irq:
    if (eadev->irq_registered) {
        free_irq(eadev->irq_line, eadev);
        eadev->irq_registered = false;
    }
err_vectors:
    pci_free_irq_vectors(pdev);
err_unmap1:
    if (eadev->bar1) {
        pci_iounmap(pdev, eadev->bar1);
        eadev->bar1 = NULL;
    }
err_unmap0:
    if (eadev->bar0) {
        pci_iounmap(pdev, eadev->bar0);
        eadev->bar0 = NULL;
    }
err_release:
    pci_release_regions(pdev);
err_free:
    kfree(eadev);
err_disable:
    pci_disable_device(pdev);
    return ret;
}

/*
 * eo_as_pci_remove: Called when the device is disconnected or driver is unloading.
 */
static void eo_as_pci_remove(struct pci_dev *pdev)
{
    struct eo_as_device *eadev = pci_get_drvdata(pdev);
    if (!eadev)
        return;

    /* disable interrupts in FPGA */
    hal_interrupt_disable(&eadev->hal);

    if (eadev->irq_registered) {
        free_irq(eadev->irq_line, eadev);
        eadev->irq_registered = false;
    }
    pci_free_irq_vectors(pdev); /* free MSI vectors if we allocated them */

    if (eadev->bar1) {
        pci_iounmap(pdev, eadev->bar1);
        eadev->bar1 = NULL;
    }
    if (eadev->bar0) {
        pci_iounmap(pdev, eadev->bar0);
        eadev->bar0 = NULL;
    }
    pci_release_regions(pdev);
    device_destroy(eadev->class, eadev->devt);
    class_destroy(eadev->class);
    cdev_del(&eadev->cdev);
    unregister_chrdev_region(eadev->devt, 1);

    /* Free DMA buffers and context */
    if (eadev->dma_ctx) {
        eo_as_free_dma_buffers(eadev->dma_ctx);
        kfree(eadev->dma_ctx->channels);
        kfree(eadev->dma_ctx);
    }

    kfree(eadev);
    pci_disable_device(pdev);

    pr_info("eo_as_fpga_driver: remove done\n");
}

/**
 * @brief Open routine for the char device.
 *
 * Associates the driver context with the opened file.
 *
 * @param inode Pointer to the inode.
 * @param filp Pointer to the file structure.
 *
 * @return 0 on success.
 */
static int eo_as_dev_open(struct inode *inode, struct file *filp)
{
    struct eo_as_device *eadev;
    eadev = container_of(inode->i_cdev, struct eo_as_device, cdev);
    filp->private_data = eadev;

    pr_info("eo_as_fpga_driver: open\n");
    return 0; 
}

/**
 * @brief Release routine for the char device.
 *
 * @param inode Pointer to the inode.
 * @param filp Pointer to the file structure.
 *
 * @return 0 on success.
 */
static int eo_as_dev_release(struct inode *inode, struct file *filp)
{
    pr_info("eo_as_fpga_driver: release\n");
    return 0;
}

/**
 * @brief Read routine for the char device.
 *
 * Reads a 32-bit FPGA register at offset 0x100, formats it in hexadecimal, and copies
 * the result to user space.
 *
 * @param filp Pointer to the file structure.
 * @param buf User-space buffer.
 * @param count Number of bytes to read.
 * @param ppos Pointer to the file offset.
 *
 * @return Number of bytes read, or a negative error code.
 */
static ssize_t eo_as_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    struct eo_as_device *eadev = filp->private_data;
    u32 val;
    char tmp[16];
    int len;

    val = hal_mmio_bar_read32(&eadev->hal, 0, 0x100);
    len = snprintf(tmp, sizeof(tmp), "0x%08x\n", val);
    if (len > count)
        return -EINVAL;
    if (copy_to_user(buf, tmp, len))
        return -EFAULT;
    return len;
}

/* 
 * Example write: parse a hex string from user, then write to FPGA reg offset 0x100.
 */
static ssize_t eo_as_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
    struct eo_as_device *eadev = filp->private_data;
    char tmp[16];
    unsigned long val;

    if (count >= sizeof(tmp))
        return -EINVAL;
    if (copy_from_user(tmp, buf, count))
        return -EFAULT;
    tmp[count] = '\0';
    if (kstrtoul(tmp, 16, &val))
        return -EINVAL;

    hal_mmio_bar_write32(&eadev->hal, 0, 0x100, (u32)val);
    pr_info("eo_as_fpga_driver: wrote 0x%x to offset 0x100\n", (u32)val);
    return count;
}

/**
 * @brief Module initialization routine.
 *
 * Registers the char device region and PCI driver.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int __init eo_as_fpga_init(void)
{
    int ret;
    dev_t devno = MKDEV(eo_as_driver_major, 0);

    pr_info("eo_as_fpga_driver: init\n");

    /* register the char device region for major=eo_as_driver_major */
    ret = register_chrdev_region(devno, 1, EO_AS_DRIVER_NAME);
    if (ret) {
        pr_err("eo_as_fpga_driver: register_chrdev_region failed: %d\n", ret);
        return ret;
    }

    ret = pci_register_driver(&eo_as_pci_driver);
    if (ret) {
        pr_err("eo_as_fpga_driver: pci_register_driver failed: %d\n", ret);
        unregister_chrdev_region(devno, 1);
    }
    return ret;
}

/**
 * @brief Module exit routine.
 *
 * Unregisters the PCI driver and char device region.
 */
static void __exit eo_as_fpga_exit(void)
{
    dev_t devno = MKDEV(eo_as_driver_major, 0);
    pr_info("eo_as_fpga_driver: exit\n");
    pci_unregister_driver(&eo_as_pci_driver);
    unregister_chrdev_region(devno, 1);
}

module_init(eo_as_fpga_init);
module_exit(eo_as_fpga_exit);

MODULE_AUTHOR("Aplit-Soft LTD & El-Or Systems LTD");
MODULE_DESCRIPTION("Linux PCIe driver for Xilinx FPGA device with interrupts, read/write registers, and DMA buffer allocation/free");
MODULE_LICENSE("GPL");
