/**
 * @file eo_as_ioctl.c
 * @brief Linux-style IOCTL logic for the FPGA driver.
 *
 * This file is separate from the main PCI driver code, containing only
 * the ioctl-related commands. The main driver can call into the function
 * here (eo_as_dev_ioctl) from its file_operations->unlocked_ioctl.
 *
 * (C) 2023 Aplit-Soft LTD & El-Or Systems LTD. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/uaccess.h> /* copy_to_user, copy_from_user */
#include "hal_hwlayer.h"   /* If you need hal_* calls. */
#include "eo_as_device.h"   /* Definitions structs */

/*
 * Suppose your main device struct is defined in "eo_as_pci.h" something like:
 *  struct eo_as_device {
 *      spinlock_t lock;
 *      struct hal_context hal;
 *      struct eo_as_dma_params dma_params;
 *      struct eo_as_mem_map    mem_map;
 *      struct eo_as_event_data event_data;
 *      ...
 *  };
 * We'll assume we can cast filp->private_data to eo_as_device.
 */

/**
 * @brief The single IOCTL handling function, replacing the Windows EvtIoDeviceControl code.
 *
 * @param filp The file struct (from the open).
 * @param cmd  The ioctl command (e.g., EO_AS_IOC_SET_DMA_REG).
 * @param arg  The user-space pointer argument.
 *
 * @return 0 on success, negative error code on failure.
 */
long eo_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct eo_as_device *dev = filp->private_data;
    long ret = 0;

    switch (cmd) {

    case EO_AS_IOC_SET_DMA_REG: {
        struct eo_as_registry_params rp;
        if (copy_from_user(&rp, (struct eo_as_registry_params *)arg, sizeof(struct eo_as_registry_params))) {
            ret = -EFAULT;
            break;
        }

        hal_mmio_bar_write32(&dev->hal, rp.bar, rp.address, rp.value);

	pr_info("eo_as_ioctl: set_dma_reg bar %d, addr 0x%llx, value 0x%x\n",
            rp.bar, (unsigned long long)rp.address, rp.value);
        break;
    }

    case EO_AS_IOC_GET_DMA_REG: {
        struct eo_as_registry_params rp;
        if (copy_from_user(&rp, (struct eo_as_registry_params *)arg, sizeof(struct eo_as_registry_params))) {
	    pr_err("eo_as_ioctl: copy from user get_dma_reg");
            ret = -EFAULT;
            break;
        }

        rp.value = hal_mmio_bar_read32(&dev->hal, rp.bar, rp.address);

	pr_info("eo_as_ioctl: get_dma_reg bar %u, addr 0x%llx, value 0x%x\n", rp.bar, (unsigned long long)rp.address, rp.value);

        if (copy_to_user((void __user *)arg, &rp, sizeof(rp)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_GET_DMA_STATUS: {
        __u32 dma_status = 0; /* Suppose you read from a FPGA reg or dev->some_state */
        /* Example: read a status register: */
        dma_status = hal_mmio_bar_read32(&dev->hal, 0, 0x200);
        if (copy_to_user((void __user *)arg, &dma_status, sizeof(dma_status)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_DMA_PARAMS_GET: {
        /* user wants to read dev->dma_params */
        if (copy_to_user((void __user *)arg, &dev->dma_params, sizeof(dev->dma_params)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_MEM_MAP_GET: {
        /* read dev->mem_map to user. */
        if (copy_to_user((void __user *)arg, &dev->mem_map, sizeof(dev->mem_map)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_EVENT_HANDLE_SET: {
        /* In Windows code, you had ObReferenceObjectByHandle. 
           In Linux, there's no direct handle. We'll store the user-supplied data. */
        struct eo_as_event_data ev;
        if (copy_from_user(&ev, (void __user *)arg, sizeof(ev))) {
            ret = -EFAULT;
            break;
        }
        dev->event_data = ev; 
        /* Possibly do something with these 'handles' or store them as tokens. */
        break;
    }

    case EO_AS_IOC_EVENT_HANDLE_GET: {
        /* copy out dev->event_data */
        if (copy_to_user((void __user *)arg, &dev->event_data, sizeof(dev->event_data)))
            ret = -EFAULT;
        break;
    }

    default:
        pr_err("eo_as_ioctl: unknown cmd=0x%x\n", cmd);
        ret = -ENOTTY;
        break;
    }

    return ret;
}
