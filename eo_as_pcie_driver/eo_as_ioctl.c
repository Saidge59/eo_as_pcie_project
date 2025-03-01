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
#include <linux/uaccess.h>
#include "hal_hwlayer.h"
#include "eo_as_device.h"
#include <linux/eventfd.h>

long eo_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct eo_as_device *dev = filp->private_data;
    long ret = 0;

    switch (cmd)
    {

    case EO_AS_IOC_SET_DMA_REG:
    {
        struct eo_as_registry_params rp;
        if (copy_from_user(&rp, (struct eo_as_registry_params *)arg, sizeof(struct eo_as_registry_params)))
        {
            ret = -EFAULT;
            break;
        }

        hal_mmio_bar_write32(&dev->hal, rp.bar, rp.address, rp.value);

        pr_info("eo_as_ioctl: set_dma_reg bar %d, addr 0x%llx, value 0x%x\n",
                rp.bar, (unsigned long long)rp.address, rp.value);
        break;
    }

    case EO_AS_IOC_GET_DMA_REG:
    {
        struct eo_as_registry_params rp;
        if (copy_from_user(&rp, (struct eo_as_registry_params *)arg, sizeof(struct eo_as_registry_params)))
        {
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

    case EO_AS_IOC_GET_DMA_STATUS:
    {
        __u32 dma_status = 0;
        dma_status = hal_mmio_bar_read32(&dev->hal, 0, 0x200);
        if (copy_to_user((void __user *)arg, &dma_status, sizeof(dma_status)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_DMA_PARAMS_GET:
    {
        if (copy_to_user((void __user *)arg, &dev->dma_params, sizeof(dev->dma_params)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_MEM_MAP_GET:
    {
        if (copy_to_user((void __user *)arg, &dev->mem_map, sizeof(dev->mem_map)))
            ret = -EFAULT;
        break;
    }

    case EO_AS_IOC_EVENT_HANDLE_SET:
    {
        uint32_t channel;
        uint32_t descriptor;
        int event_handles[MAX_NUM_CHANNELS * MAX_NUM_DESCRIPTORS];

        if (copy_from_user(event_handles, (void __user *)arg, sizeof(event_handles)))
        {
            ret = -EFAULT;
            break;
        }

        for (channel = 0; channel < MAX_NUM_CHANNELS; ++channel)
        {
            for (descriptor = 0; descriptor < MAX_NUM_DESCRIPTORS; ++descriptor)
            {
                int index = channel * MAX_NUM_DESCRIPTORS + descriptor;
                dev->event_data.chans[channel].events[descriptor].event_handle = event_handles[index];
                dev->event_data.chans[channel].events[descriptor].event_ctx = eventfd_ctx_fdget(event_handles[index]);
            }
        }
        break;
    }

    case EO_AS_IOC_EVENT_HANDLE_GET:
    {
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
