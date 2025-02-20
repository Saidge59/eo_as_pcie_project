#ifndef EO_AS_IOCTL_H
#define EO_AS_IOCTL_H

/**
 * @brief The single IOCTL handling function, replacing the Windows EvtIoDeviceControl code.
 *
 * @param filp The file struct (from the open).
 * @param cmd  The ioctl command (e.g., EO_AS_IOC_SET_DMA_REG).
 * @param arg  The user-space pointer argument.
 *
 * @return 0 on success, negative error code on failure.
 */
long eo_as_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* EO_AS_IOCTL_H */
