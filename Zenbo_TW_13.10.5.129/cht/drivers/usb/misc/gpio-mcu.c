#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#define MCU_GPIO_NRST	288	//SE60
#define MCU_GPIO_MCU_ACK	334	//E20
#define MCU_GPIO_BOOT0	395	//N54
#define P_VDB_EN	329	//E15
#define R200_POWER	348	//N7

struct kobject *mcu_control_kobj;
bool bootloader_mode;

extern bool check_cos_mode(void);

static ssize_t bootloader_mode_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (bootloader_mode)
		return sprintf(buf, "MCU:bootloader mode\n");
	else
		return sprintf(buf, "MCU:normal boot mode\n");
}

static ssize_t bootloader_mode_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	if (buf[0] == '1') {
		bootloader_mode = true;
		gpio_direction_output(MCU_GPIO_MCU_ACK, 0);
		msleep(2000);
		gpio_direction_output(MCU_GPIO_MCU_ACK, 1);
		gpio_direction_output(MCU_GPIO_BOOT0, 1);
		gpio_direction_output(MCU_GPIO_NRST, 0);
		msleep(3);
		gpio_direction_output(MCU_GPIO_NRST, 1);
	}
	if (buf[0] == '0') {
		bootloader_mode = false;
		gpio_direction_output(MCU_GPIO_BOOT0, 0);
		gpio_direction_output(MCU_GPIO_NRST, 0);
		msleep(3);
		gpio_direction_output(MCU_GPIO_NRST, 1);
	}
	return count;
}
static struct kobj_attribute bootloader_mode_enable_attribute = __ATTR(bootloader_mode_enable, 0666,
                                bootloader_mode_enable_show, bootloader_mode_enable_store);

static ssize_t hard_reset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "Use \"echo 1 > hard_reset\" to perform MCU hard reset.\n");
}

static ssize_t hard_reset_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		gpio_direction_output(MCU_GPIO_MCU_ACK, 0);
		msleep(2000);
		gpio_direction_output(MCU_GPIO_MCU_ACK, 1);
		gpio_direction_output(MCU_GPIO_NRST, 0);
		msleep(3);
		gpio_direction_output(MCU_GPIO_NRST, 1);
	}
	return count;
}
static struct kobj_attribute hard_reset_attribute = __ATTR(hard_reset, 0666,
                                hard_reset_show, hard_reset_store);

static ssize_t driver_board_power_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "Driver board power = %s\n", gpio_get_value(P_VDB_EN)? "H":"L");
}

static ssize_t driver_board_power_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] == '1' || buf[0] == 'H')
		gpio_direction_output(P_VDB_EN, 1);
	if (buf[0] == '0' || buf[0] == 'L') {
		gpio_direction_output(MCU_GPIO_MCU_ACK, 0);
		msleep(2000);
		gpio_direction_output(MCU_GPIO_MCU_ACK, 1);
		gpio_direction_output(P_VDB_EN, 0);
	}
	return count;
}
static struct kobj_attribute driver_board_power_attribute = __ATTR(driver_board_power, 0666,
                                driver_board_power_show, driver_board_power_store);

#ifdef DEBUG_GPIO
static ssize_t gpio_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t gpio_state_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int gpio_num=0, state=0;

	sscanf(buf, "%d %d", &gpio_num, &state);
	printk("gpio_num = %d, state = %d", gpio_num, state);
	gpio_direction_output(gpio_num, state);
	return count;
}
static struct kobj_attribute gpio_state_attribute = __ATTR(gpio_state, 0666,
                                gpio_state_show, gpio_state_store);
#endif

static int __init mcu_gpio_init(void)
{
	int ret = 0;

	bootloader_mode = false;

	pr_info("%s\n", __func__);
	if (check_cos_mode()) {
		ret = gpio_request(R200_POWER, "r200_power");
		if (ret)
			pr_info("gpio %d request failed\n", R200_POWER);

		pr_info("Turn off R200 power\n");
		ret = gpio_direction_output(R200_POWER, 0);
		if (ret)
			pr_info("gpio %d unavaliable for output\n", R200_POWER);
	}

	ret = gpio_request(P_VDB_EN, "driver_board_power");
	if (ret){
		printk(KERN_INFO"gpio %d request failed \n", P_VDB_EN);
	}
	ret = gpio_request(MCU_GPIO_BOOT0, "mcu_boot0");
	if (ret){
		printk(KERN_INFO"gpio %d request failed \n", MCU_GPIO_BOOT0);
		goto err_gpio;
	}
	ret = gpio_request(MCU_GPIO_MCU_ACK, "mcu_ack");
	if (ret){
		printk(KERN_INFO"gpio %d request failed \n", MCU_GPIO_MCU_ACK);
		goto err_gpio;
	}
	ret = gpio_request(MCU_GPIO_NRST, "mcu_nrst");
	if (ret){
		printk(KERN_INFO"gpio %d request failed \n", MCU_GPIO_NRST);
		goto err_gpio;
	}

	ret = gpio_direction_output(MCU_GPIO_BOOT0, 0);
	if (ret) {
		printk(KERN_INFO"gpio %d unavaliable for output \n", MCU_GPIO_BOOT0);
		goto err_free_gpio;
	}
	ret = gpio_direction_output(MCU_GPIO_MCU_ACK, 1);
        if (ret) {
                printk(KERN_INFO"gpio %d unavaliable for output \n", MCU_GPIO_MCU_ACK);
                goto err_free_gpio;
        }
	ret = gpio_direction_output(MCU_GPIO_NRST, 1);
	if (ret) {
		printk(KERN_INFO"gpio %d unavaliable for output \n", MCU_GPIO_NRST);
		goto err_free_gpio;
	}

	printk(KERN_INFO "GPIO pin requested ok, MCU_GPIO_BOOT0 = %s\n", gpio_get_value(MCU_GPIO_BOOT0)? "H":"L");
	printk(KERN_INFO "GPIO pin requested ok, MCU_GPIO_MCU_ACK = %s\n", gpio_get_value(MCU_GPIO_MCU_ACK)? "H":"L");
	printk(KERN_INFO "GPIO pin requested ok, MCU_GPIO_NRST = %s\n", gpio_get_value(MCU_GPIO_NRST)? "H":"L");

	mcu_control_kobj = kobject_create_and_add("mcu", NULL);
	if (mcu_control_kobj == NULL)
	{
		printk(KERN_INFO "%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(mcu_control_kobj, &driver_board_power_attribute);
	if (ret)
	{
		printk(KERN_INFO "%s: sysfs_create_file: driver_board_power failed\n", __func__);
		return ret;
	}

#ifdef DEBUG_GPIO
	ret = sysfs_create_file(mcu_control_kobj, &gpio_state_attribute);
	if (ret)
	{
		printk(KERN_INFO "%s: sysfs_create_file: gpio_state failed\n", __func__);
		return ret;
	}
#endif

	ret = sysfs_create_file(mcu_control_kobj, &bootloader_mode_enable_attribute);
	if (ret)
	{
		printk(KERN_INFO "%s: sysfs_create_file: bootloader_mode_enable failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(mcu_control_kobj, &hard_reset_attribute);
	if (ret)
	{
		printk(KERN_INFO "%s: sysfs_create_file: hard_reset failed\n", __func__);
		return ret;
	}

	return 0;

err_free_gpio:
	if (gpio_is_valid(MCU_GPIO_BOOT0))
		gpio_free(MCU_GPIO_BOOT0);
	if (gpio_is_valid(MCU_GPIO_MCU_ACK))
		gpio_free(MCU_GPIO_MCU_ACK);
	if (gpio_is_valid(MCU_GPIO_NRST))
		gpio_free(MCU_GPIO_NRST);
err_gpio:
	return ret;

}

static void __exit mcu_gpio_exit(void)
{
	if (gpio_is_valid(P_VDB_EN))
		gpio_free(P_VDB_EN);
	if (gpio_is_valid(MCU_GPIO_NRST))
		gpio_free(MCU_GPIO_NRST);
	if (gpio_is_valid(MCU_GPIO_MCU_ACK))
		gpio_free(MCU_GPIO_MCU_ACK);
	if (gpio_is_valid(MCU_GPIO_BOOT0))
		gpio_free(MCU_GPIO_BOOT0);

	kobject_put(mcu_control_kobj);
}

fs_initcall(mcu_gpio_init);
module_exit(mcu_gpio_exit);

MODULE_AUTHOR("The ASUS BSP team");
MODULE_DESCRIPTION("GPIO driver for MCU Board");
MODULE_LICENSE("GPL");
