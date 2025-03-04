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
#include <linux/uaccess.h>
#include <linux/eventfd.h>
#include "eo_as_ioctl.h"
#include "eo_as_device.h"
#include "eo_as_dma.h"
#include "hal_hwlayer.h"
#include "lin_isr.h"
#include "public.h"

#define EO_AS_VENDOR_ID 0x10EE
#define EO_AS_DEVICE_ID 0x9011

#define EO_AS_DRIVER_NAME "eo_as_fpga_driver"
static int eo_as_driver_major = 270;

/**
 * @brief Probe routine for the PCI driver.
 *
 * Called when the PCI core finds a device matching our device ID table.
 *
 * @param pdev Pointer to the PCI device.
 * @param ent Pointer to the matching PCI device ID entry.
 * @return 0 on success, or a negative error code on failure.
 */
static int eo_as_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent);

/**
 * @brief Remove routine for the PCI driver.
 *
 * Called when the device is disconnected or the driver is unloading.
 *
 * @param pdev Pointer to the PCI device.
 */
static void eo_as_pci_remove(struct pci_dev *pdev);

/**
 * @brief Open routine for the char device.
 *
 * Associates the driver context with the opened file.
 *
 * @param inode Pointer to the inode.
 * @param filp Pointer to the file structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int eo_as_dev_open(struct inode *inode, struct file *filp);

/**
 * @brief Release routine for the char device.
 *
 * Cleans up resources associated with the file.
 *
 * @param inode Pointer to the inode.
 * @param filp Pointer to the file structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int eo_as_dev_release(struct inode *inode, struct file *filp);

/**
 * @brief Read routine for the char device.
 *
 * Reads data from the FPGA and copies it to user space.
 *
 * @param filp Pointer to the file structure.
 * @param buf User-space buffer to store the data.
 * @param count Number of bytes to read.
 * @param ppos Pointer to the file offset.
 * @return Number of bytes read, or a negative error code on failure.
 */
static ssize_t eo_as_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);

/**
 * @brief Write routine for the char device.
 *
 * Writes data from user space to the FPGA.
 *
 * @param filp Pointer to the file structure.
 * @param buf User-space buffer containing the data.
 * @param count Number of bytes to write.
 * @param ppos Pointer to the file offset.
 * @return Number of bytes written, or a negative error code on failure.
 */
static ssize_t eo_as_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos);

/**
 * @brief Memory mapping routine for the char device.
 *
 * Maps DMA buffers to user-space virtual memory.
 *
 * @param filp Pointer to the file structure.
 * @param vma Pointer to the virtual memory area structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int eo_as_dev_mmap(struct file *filp, struct vm_area_struct *vma);

/**
 * @brief PCI device ID table for the driver.
 *
 * Defines the vendor and device IDs that this driver supports. The table is terminated
 * with an entry containing all zeros.
 */
static const struct pci_device_id eo_as_pci_ids[] = {
    {PCI_DEVICE(EO_AS_VENDOR_ID, EO_AS_DEVICE_ID)},
    {
        0,
    }};
MODULE_DEVICE_TABLE(pci, eo_as_pci_ids);

/**
 * @brief PCI driver structure for the FPGA device.
 *
 * Contains the driver's name, supported device IDs, and function pointers for
 * probing and removing the device.
 */
static struct pci_driver eo_as_pci_driver = {
    .name = EO_AS_DRIVER_NAME,
    .id_table = eo_as_pci_ids,
    .probe = eo_as_pci_probe,
    .remove = eo_as_pci_remove,
};

/**
 * @brief File operations structure for the char device.
 *
 * Defines the function pointers for handling file operations such as open, release,
 * read, write, ioctl, and memory mapping.
 */
static const struct file_operations eo_as_dev_fops = {
    .owner = THIS_MODULE,
    .open = eo_as_dev_open,
    .release = eo_as_dev_release,
    .read = eo_as_dev_read,
    .write = eo_as_dev_write,
    .unlocked_ioctl = eo_as_dev_ioctl,
    .mmap = eo_as_dev_mmap,
};

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

    ret = pci_enable_device(pdev);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: pci_enable_device failed: %d\n", ret);
        return ret;
    }

    eadev = kzalloc(sizeof(*eadev), GFP_KERNEL);
    if (!eadev)
    {
        ret = -ENOMEM;
        goto err_disable;
    }
    eadev->pdev = pdev;
    pci_set_drvdata(pdev, eadev);

    spin_lock_init(&eadev->lock);

    pci_set_master(pdev);

    ret = pci_request_regions(pdev, EO_AS_DRIVER_NAME);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: pci_request_regions failed: %d\n", ret);
        goto err_free;
    }

    bar_start0 = pci_resource_start(pdev, 0);
    bar_len0 = pci_resource_len(pdev, 0);
    if (!bar_start0 || !bar_len0)
    {
        pr_err("eo_as_fpga_driver: invalid BAR0\n");
        ret = -ENODEV;
        goto err_release;
    }

    bar_start1 = pci_resource_start(pdev, 2);
    bar_len1 = pci_resource_len(pdev, 2);
    if (!bar_start1 || !bar_len1)
    {
        pr_err("eo_as_fpga_driver: invalid BAR1\n");
        ret = -ENODEV;
        goto err_release;
    }

    eadev->bar0 = pci_iomap(pdev, 0, 0);
    if (!eadev->bar0)
    {
        pr_err("eo_as_fpga_driver: pci_iomap bar0 fail\n");
        ret = -ENOMEM;
        goto err_release;
    }
    eadev->bar1 = pci_iomap(pdev, 2, 0);
    if (!eadev->bar1)
    {
        pr_err("eo_as_fpga_driver: pci_iomap bar1 fail\n");
        ret = -ENOMEM;
        goto err_unmap0;
    }

    eadev->hal.bar[0] = eadev->bar0;
    eadev->hal.bar_length[0] = bar_len0;
    eadev->hal.bar[1] = eadev->bar1;
    eadev->hal.bar_length[1] = bar_len1;
    eadev->hal.num_bars = 2;

    ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_MSIX | PCI_IRQ_LEGACY);
    if (ret < 0)
    {
        pr_err("eo_as_fpga_driver: pci_alloc_irq_vectors failed: %d\n", ret);
        goto err_unmap1;
    }
    vector = pci_irq_vector(pdev, 0);
    ret = request_irq(vector, eo_as_irq_handler, 0,
                      EO_AS_DRIVER_NAME, eadev);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: request_irq (vector %d) failed: %d\n", vector, ret);
        goto err_vectors;
    }
    eadev->irq_registered = true;
    eadev->irq_line = vector;

    eadev->dma_ctx = (struct dma_device_context *)kzalloc(sizeof(struct dma_device_context), GFP_KERNEL);
    if (!eadev->dma_ctx)
    {
        ret = -ENOMEM;
        goto err_irq;
    }
    eadev->dma_ctx->dev = &pdev->dev;
    eadev->dma_ctx->num_channels = MAX_NUM_CHANNELS;
    eadev->dma_ctx->channels = kzalloc(sizeof(struct dma_channel) * MAX_NUM_CHANNELS, GFP_KERNEL);
    if (!eadev->dma_ctx->channels)
    {
        ret = -ENOMEM;
        goto err_free_dma_ctx;
    }

    ret = eo_as_allocate_dma_buffers(eadev);
    if (ret < 0)
        goto err_free_dma_channels;

    ret = alloc_chrdev_region(&devno, 0, 1, EO_AS_DRIVER_NAME);
    if (ret < 0)
    {
        pr_err("eo_as_fpga_driver: alloc_chrdev_region failed: %d\n", ret);
        goto err_irq;
    }
    eadev->devt = devno;

    cdev_init(&eadev->cdev, &eo_as_dev_fops);
    eadev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&eadev->cdev, eadev->devt, 1);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: cdev_add failed: %d\n", ret);
        goto err_register_chrdev_region;
    }

    eadev->class = class_create(THIS_MODULE, EO_AS_DRIVER_NAME);
    if (IS_ERR(eadev->class))
    {
        ret = PTR_ERR(eadev->class);
        goto err_cdev;
    }

    eadev->dev = device_create(eadev->class, NULL, eadev->devt, NULL, EO_AS_DRIVER_NAME);
    if (IS_ERR(eadev->dev))
    {
        ret = PTR_ERR(eadev->dev);
        class_destroy(eadev->class);
        goto err_class_destroy;
    }

    eadev->dma_params.dma_desc_count = MAX_NUM_DESCRIPTORS;
    eadev->dma_params.dma_buf_size = DESCRIPTOR_BUF_SIZE;
    eadev->dma_params.dma_chan_count = MAX_NUM_CHANNELS;

    eo_as_interrupt_enable(eadev);

    pr_info("eo_as_fpga_driver: probe success, major=%d minor=%d, bar0=0x%llx, bar1=0x%llx, MSI vector=%d\n",
            MAJOR(eadev->devt), MINOR(eadev->devt),
            (unsigned long long)bar_start0,
            (unsigned long long)bar_start1,
            vector);
    return 0;


err_class_destroy:
    class_destroy(eadev->class);
err_cdev:
    cdev_del(&eadev->cdev);
err_register_chrdev_region:
    unregister_chrdev_region(eadev->devt, 1);
err_free_dma_channels:
    eo_as_free_dma_buffers(eadev);
    kfree(eadev->dma_ctx->channels);
err_free_dma_ctx:
    kfree(eadev->dma_ctx);
err_irq:
    if (eadev->irq_registered)
    {
        free_irq(eadev->irq_line, eadev);
        eadev->irq_registered = false;
    }
err_vectors:
    pci_free_irq_vectors(pdev);
err_unmap1:
    if (eadev->bar1)
    {
        pci_iounmap(pdev, eadev->bar1);
        eadev->bar1 = NULL;
    }
err_unmap0:
    if (eadev->bar0)
    {
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

static void eo_as_pci_remove(struct pci_dev *pdev)
{
    struct eo_as_device *eadev = pci_get_drvdata(pdev);
    if (!eadev)
        return;

    eo_as_interrupt_disable(eadev);
    if (eadev->irq_registered)
    {
        free_irq(eadev->irq_line, eadev);
        eadev->irq_registered = false;
    }
    pci_free_irq_vectors(pdev);

    if (eadev->bar1)
    {
        pci_iounmap(pdev, eadev->bar1);
        eadev->bar1 = NULL;
    }
    if (eadev->bar0)
    {
        pci_iounmap(pdev, eadev->bar0);
        eadev->bar0 = NULL;
    }
    pci_release_regions(pdev);
    device_destroy(eadev->class, eadev->devt);
    class_destroy(eadev->class);
    cdev_del(&eadev->cdev);
    unregister_chrdev_region(eadev->devt, 1);

    if (eadev->dma_ctx)
    {
        eo_as_free_dma_buffers(eadev);
        kfree(eadev->dma_ctx->channels);
        kfree(eadev->dma_ctx);
    }

    kfree(eadev);
    pci_disable_device(pdev);

    pr_info("eo_as_fpga_driver: remove done\n");
}

static int eo_as_dev_open(struct inode *inode, struct file *filp)
{
    struct eo_as_device *eadev;
    eadev = container_of(inode->i_cdev, struct eo_as_device, cdev);
    filp->private_data = eadev;

    pr_info("eo_as_fpga_driver: open\n");
    return 0;
}

static int eo_as_dev_release(struct inode *inode, struct file *filp)
{
    int channel;
    int descriptor;
    struct eventfd_ctx *event_ctx;
    struct eo_as_device *eadev;
    eadev = container_of(inode->i_cdev, struct eo_as_device, cdev);

    for (channel = 0; channel < MAX_NUM_CHANNELS; channel++)
    {
        for (descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; descriptor++)
        {
            event_ctx = eadev->event_data.chans[channel].events[descriptor].event_ctx;
            if (event_ctx)
            {
                eventfd_ctx_put(event_ctx);
                eadev->event_data.chans[channel].events[descriptor].event_ctx = NULL;
            }
        }
    }

    pr_info("eo_as_fpga_driver: release\n");
    return 0;
}

static ssize_t eo_as_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    u32 val;
    char tmp[16];
    int len;
    struct eo_as_device *eadev = filp->private_data;
    if (!eadev)
    {
        pr_err("eo_as_fpga_driver 'read': Not exist private_data\n");
        return -EINVAL;
    }

    val = hal_mmio_bar_read32(&eadev->hal, 0, 0x100);
    len = snprintf(tmp, sizeof(tmp), "0x%08x\n", val);
    if (len > count)
        return -EINVAL;
    if (copy_to_user(buf, tmp, len))
        return -EFAULT;
    return len;
}

static ssize_t eo_as_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
    char tmp[16];
    unsigned long val;
    struct eo_as_device *eadev = filp->private_data;
    if (!eadev)
    {
        pr_err("eo_as_fpga_driver 'write': Not exist private_data\n");
        return -EINVAL;
    }

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

static int eo_as_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct eo_as_device *eadev = filp->private_data;
    dma_addr_t dma_handle;
    int channel, descriptor;
    size_t total_size = MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS * DESCRIPTOR_BUF_SIZE;
    size_t requested_size = vma->vm_end - vma->vm_start;
    unsigned long offset = 0;
    void *buffer;
    struct vm_area_struct tmp_vma;
    int ret;

    if (!eadev)
    {
        pr_err("eo_as_fpga_driver 'mmap': No private_data\n");
        return -EINVAL;
    }

    if (requested_size != total_size)
    {
        pr_err("eo_as_fpga_driver 'mmap': Requested size (%zu) does not match total size (%zu)\n",
               requested_size, total_size);
        return -EINVAL;
    }

    for (channel = 0; channel < MAX_NUM_CHANNELS; channel++)
    {
        for (descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; descriptor++)
        {            
            dma_handle = eadev->dma_ctx->channels[channel].descriptors[descriptor].dma_handle;
            buffer = eadev->dma_ctx->channels[channel].descriptors[descriptor].buffer;

            tmp_vma = *vma;
            tmp_vma.vm_start = vma->vm_start + offset;
            tmp_vma.vm_end = tmp_vma.vm_start + DESCRIPTOR_BUF_SIZE;

            ret = dma_mmap_coherent(eadev->dma_ctx->dev, &tmp_vma, buffer, dma_handle, DESCRIPTOR_BUF_SIZE);
            if (ret) {
                pr_err("ch %d desc %d: dma_mmap_coherent failed: %d\n", channel, descriptor, ret);
                eo_as_interrupt_enable(eadev);
                return ret;
            }

            offset += DESCRIPTOR_BUF_SIZE;
        }
    }

    pr_info("eo_as_dev_mmap: Mapped %d channels, %d descriptors successfully, total size=%zu\n",
            MAX_NUM_CHANNELS, MAX_NUM_DESCRIPTORS, total_size);

    return 0;
}

static int __init eo_as_fpga_init(void)
{
    int ret;
    dev_t devno = MKDEV(eo_as_driver_major, 0);

    pr_info("eo_as_fpga_driver: init\n");

    ret = register_chrdev_region(devno, 1, EO_AS_DRIVER_NAME);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: register_chrdev_region failed: %d\n", ret);
        return ret;
    }

    ret = pci_register_driver(&eo_as_pci_driver);
    if (ret)
    {
        pr_err("eo_as_fpga_driver: pci_register_driver failed: %d\n", ret);
        unregister_chrdev_region(devno, 1);
    }
    return ret;
}

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
