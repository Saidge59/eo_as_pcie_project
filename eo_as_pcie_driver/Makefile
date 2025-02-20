# This must be in the same directory as your .c files
# so the kernel build system knows what to build.

obj-m += eo_as_pci_driver.o

# If your driver is split into multiple .c files, specify them:
eo_as_pci_driver-objs := eo_as_pcie.o eo_as_dma.o eo_as_ioctl.o lin_isr.o hal_hwlayer.o

ccflags-y += -I$(src)/../include
