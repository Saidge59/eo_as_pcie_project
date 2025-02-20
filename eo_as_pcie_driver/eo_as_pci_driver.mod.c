#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x8aa1adfc, "module_layout" },
	{ 0x2e3a7d93, "cdev_del" },
	{ 0x8f151a2c, "kmalloc_caches" },
	{ 0xee789116, "cdev_init" },
	{ 0x1fdc7df2, "_mcount" },
	{ 0xc11613e7, "pci_free_irq_vectors" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x3fd78f3b, "register_chrdev_region" },
	{ 0x4d17c01, "pci_disable_device" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xadf38724, "device_destroy" },
	{ 0xd1568e0e, "pci_release_regions" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x43779f9e, "dma_free_attrs" },
	{ 0xb9cd0e10, "pci_set_master" },
	{ 0xccb5e580, "pci_alloc_irq_vectors_affinity" },
	{ 0x5c3c7387, "kstrtoull" },
	{ 0xdcb764ad, "memset" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0x40b02127, "pci_iounmap" },
	{ 0x9d2ab8ac, "__tasklet_schedule" },
	{ 0x224ab02f, "dma_alloc_attrs" },
	{ 0x774854c, "device_create" },
	{ 0x2364c85a, "tasklet_init" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x877cf29f, "_dev_err" },
	{ 0x2ac7ef9d, "cdev_add" },
	{ 0xbbf7d725, "_dev_info" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0x92997ed8, "_printk" },
	{ 0x908e5601, "cpu_hwcaps" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0x217b7e93, "pci_unregister_driver" },
	{ 0xe18001b3, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xce9d80be, "pci_irq_vector" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0xbc85260f, "pci_request_regions" },
	{ 0x44992b2, "__pci_register_driver" },
	{ 0x6c4d42dc, "class_destroy" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xbb1c9eae, "pci_iomap" },
	{ 0xd1a489b3, "pci_enable_device" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xab430bfd, "__class_create" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v000010EEd00009011sv*sd*bc*sc*i*");
