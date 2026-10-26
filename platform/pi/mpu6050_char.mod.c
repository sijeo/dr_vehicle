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
	{ 0x7bc0121e, "param_ops_int" },
	{ 0x41ab58bf, "i2c_del_driver" },
	{ 0x8ccaef41, "i2c_register_driver" },
	{ 0xae353d77, "arm_copy_from_user" },
	{ 0x49970de8, "finish_wait" },
	{ 0x647af474, "prepare_to_wait_event" },
	{ 0x1000e51, "schedule" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x13d0adf7, "__kfifo_out" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x51a910c0, "arm_copy_to_user" },
	{ 0x7441a91a, "sysfs_remove_group" },
	{ 0xa5dda2bb, "devm_request_threaded_irq" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0xc7885d0b, "cdev_del" },
	{ 0x26d1dbff, "class_destroy" },
	{ 0x2a0c0a51, "device_destroy" },
	{ 0x5418143d, "sysfs_create_group" },
	{ 0xf3725a4c, "device_create" },
	{ 0x59d82f5, "__class_create" },
	{ 0xdd0a9799, "cdev_add" },
	{ 0xc38969c2, "cdev_init" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0x12a38747, "usleep_range" },
	{ 0x3a349840, "device_property_read_u32_array" },
	{ 0x5bbe49f4, "__init_waitqueue_head" },
	{ 0xe346f67a, "__mutex_init" },
	{ 0x9f61ee91, "__devm_regmap_init_i2c" },
	{ 0x46fe63d0, "devm_kmalloc" },
	{ 0x59e5070d, "__do_div64" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0x8c8569cb, "kstrtoint" },
	{ 0x3b6c41ea, "kstrtouint" },
	{ 0x3dcf1ffa, "__wake_up" },
	{ 0xf23fcb99, "__kfifo_in" },
	{ 0xdb9ca3c5, "_raw_spin_lock" },
	{ 0x5f754e5a, "memset" },
	{ 0xee43fd9b, "___ratelimit" },
	{ 0xe6dff851, "i2c_smbus_read_i2c_block_data" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xdcd590b1, "regmap_bulk_read" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0x5af2e898, "kmem_cache_alloc_trace" },
	{ 0x52afd999, "kmalloc_caches" },
	{ 0xb8855baa, "_dev_err" },
	{ 0x37a0cba, "kfree" },
	{ 0x67ea780, "mutex_unlock" },
	{ 0xc271c3be, "mutex_lock" },
	{ 0x9bdbf16f, "regmap_write" },
	{ 0xb2562e02, "regmap_update_bits_base" },
	{ 0x314b20c8, "scnprintf" },
	{ 0x3a188f81, "_dev_info" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0xf2a4c553, "_dev_warn" },
	{ 0x160a9d89, "regmap_read" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "regmap-i2c");

MODULE_ALIAS("i2c:mpu6050");
MODULE_ALIAS("of:N*T*Cstm,ism330dlctr");
MODULE_ALIAS("of:N*T*Cstm,ism330dlctrC*");
MODULE_ALIAS("of:N*T*Cst,ism330dlc");
MODULE_ALIAS("of:N*T*Cst,ism330dlcC*");
MODULE_ALIAS("of:N*T*Cst,ism330bx");
MODULE_ALIAS("of:N*T*Cst,ism330bxC*");
MODULE_ALIAS("of:N*T*Cinvensense,mpu6050-custom");
MODULE_ALIAS("of:N*T*Cinvensense,mpu6050-customC*");

MODULE_INFO(srcversion, "D8F9FE2FE871E58CD8ADEAE");
