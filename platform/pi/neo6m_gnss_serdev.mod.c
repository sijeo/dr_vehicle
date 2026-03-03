#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
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
__used __section(__versions) = {
	{ 0xfea6deb5, "module_layout" },
	{ 0xc01269cd, "driver_unregister" },
	{ 0x86bf4310, "__serdev_device_driver_register" },
	{ 0x84b183ae, "strncmp" },
	{ 0x37a0cba, "kfree" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x8f3625fe, "_raw_spin_unlock_bh" },
	{ 0x2b5ab97d, "_raw_spin_lock_bh" },
	{ 0x85df9b6c, "strsep" },
	{ 0x2d39b0a7, "kstrdup" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xf2a4c553, "_dev_warn" },
	{ 0xdd27fa87, "memchr" },
	{ 0xb742fd7, "simple_strtol" },
	{ 0x73e20c1c, "strlcpy" },
	{ 0x328a05f1, "strncpy" },
	{ 0x349cba85, "strchr" },
	{ 0x97255bdf, "strlen" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x51a910c0, "arm_copy_to_user" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xb8855baa, "_dev_err" },
	{ 0x5418143d, "sysfs_create_group" },
	{ 0xf3725a4c, "device_create" },
	{ 0xdd0a9799, "cdev_add" },
	{ 0xc38969c2, "cdev_init" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0x59d82f5, "__class_create" },
	{ 0x8faf04c0, "serdev_device_set_flow_control" },
	{ 0xc42c8330, "serdev_device_set_baudrate" },
	{ 0x6b4f406d, "serdev_device_open" },
	{ 0x5f754e5a, "memset" },
	{ 0x46fe63d0, "devm_kmalloc" },
	{ 0xb2d48a2e, "queue_work_on" },
	{ 0x9d669763, "memcpy" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x314b20c8, "scnprintf" },
	{ 0x39a12ca7, "_raw_spin_unlock_irqrestore" },
	{ 0x5f849a69, "_raw_spin_lock_irqsave" },
	{ 0x3a188f81, "_dev_info" },
	{ 0x4205ad24, "cancel_work_sync" },
	{ 0x2fb4628a, "serdev_device_close" },
	{ 0x26d1dbff, "class_destroy" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0xc7885d0b, "cdev_del" },
	{ 0x2a0c0a51, "device_destroy" },
	{ 0x7441a91a, "sysfs_remove_group" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Cublox,neo-6m");
MODULE_ALIAS("of:N*T*Cublox,neo-6mC*");

MODULE_INFO(srcversion, "BD5F08297D2748558BE8BCD");
