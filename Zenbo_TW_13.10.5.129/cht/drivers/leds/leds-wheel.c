#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/board_asustek.h>
#include <linux/power/bq30z55_battery.h>

#include "leds-wheel.h"

static struct class *leds_wheel_class;
static int tmp_wheel_num;
static int led_state = IDLE_STATE;

static struct workqueue_struct *leds_wq;

static int start_fw_update(struct led_wheel_data *led_data, char *fw_name, int check_cs);
static int loader_get_checksum(struct led_wheel_data *led_data, int type);
static int get_bootloader_ver(struct led_wheel_data *led_data, u8 *buf);
static int get_app_ver(struct led_wheel_data *led_data, u8 *buf);
static void deep_power_reset(struct led_wheel_data *led_data);
static void power_reset(struct led_wheel_data *led_data);
static void i2c_reset(struct led_wheel_data *led_data);

static int i2c_write(struct i2c_client *client, int wlength, unsigned char *wdata)
{
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;//Write
	msg[0].len = wlength;
	msg[0].buf = (unsigned char *)wdata;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret >= 0 && ret != 1) {
		LED_ERR("msg size is wrong, %d\n", ret);
		ret = -ECOMM;
	}

	return ret;
}

static int i2c_read(struct i2c_client *client, int rlength, unsigned char *rdata)
{
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD;//Read
	msg[0].len = rlength;
	msg[0].buf = (unsigned char *)rdata;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret >= 0 && ret != 1) {
		LED_ERR("msg size is wrong, %d\n", ret);
		ret = -ECOMM;
	}

	return ret;
}

static int is_hex(char num)
{
	//0-9, a-f, A-F
	if ((47 < num && num < 58) || (64 < num && num < 71) || (96 < num && num < 103))
		return 1;
	return 0;
}
static int string_to_byte(const char *source, unsigned char *destination, int size)
{
	int i = 0,counter = 0;
	char c[3] = {0};
	unsigned char bytes;

	if (size%2 == 1)
		return -EINVAL;

	for(i = 0; i < size; i++){
		if(!is_hex(source[i])) {
			LED_ERR("input includes a non-hex number\n");
			return -EINVAL;
		}
		if(0 == i%2){
			c[0] = source[i];
			c[1] = source[i+1];
			sscanf(c, "%hhx", &bytes);
			destination[counter] = bytes;
			counter++;
		}
	}
	return 0;
}

static u16 calculate_checksum(const u8 *buf, int size)
{
	u16 checksum=0;
	int i=0;

	for (i=0; i < size; i++) {
		checksum += buf[i];
	}
	return checksum;
}

static int try_reset(int error)
{
	switch(error) {
	case -ETIMEDOUT:
		return 1;
	default:
		break;
	}
	return 0;
}

static int send_raw_cmd(struct led_wheel_data *led_data, const char *cmd, int size)
{
	unsigned char raw_cmd[NORMAL_WRITE_SIZE];
	int ret = 0, retry = 0, recovery = 0;

	if (size != NORMAL_WRITE_SIZE*2) {
		LED_ERR("command size is not %d\n", NORMAL_WRITE_SIZE*2);
		return -EINVAL;
	}

	string_to_byte(cmd, raw_cmd, NORMAL_WRITE_SIZE*2);

	LED_DBG("Hex command(%d):%02X %02X %02X %02X %02X %02X %02X\n", led_data->wheel_num,
			raw_cmd[0],  raw_cmd[1], raw_cmd[2], raw_cmd[3], raw_cmd[4], raw_cmd[5], raw_cmd[6]);

	while (retry <= RETRY_COUNT_SENDCMD) {
		ret = i2c_write(led_data->client, NORMAL_WRITE_SIZE, raw_cmd);
		if (ret > 0) {
			if (recovery == 1) {
				LED_INFO("recovred by i2c reset\n");
				ret = -ERROR_CODE_I2C_RECOVERY;
			} else if (recovery == 2) {
				LED_INFO("recovred by power reset\n");
				ret = -ERROR_CODE_POWER_RECOVERY;
			}
			break;
		}

		retry++;
		LED_ERR("wheel(%d) i2c_write(%02X%02X%02X%02X%02X%02X%02X) failed(%d), try %d\n",
				led_data->wheel_num, raw_cmd[0],  raw_cmd[1], raw_cmd[2], raw_cmd[3],
				raw_cmd[4], raw_cmd[5], raw_cmd[6], ret, retry);

		/* Try to reset when i2c timeout, or second retry. */
		if (try_reset(ret) || retry > 1) {
			if (retry < RETRY_COUNT_SENDCMD) {
				LED_ERR("i2c reset!\n");
				i2c_reset(led_data);
				msleep(50);
				recovery = 1;
			} else if (retry == RETRY_COUNT_SENDCMD){
				LED_ERR("deep power reset!\n");
				deep_power_reset(led_data);
				msleep(50);
				recovery = 2;
			}
		}
		msleep(50);
	}

	return ret;
}

static int leds_loader_write(struct i2c_client *client, u8 cmd, const u8 *buf, u8 size, u8 checksum)
{
	u8 byte_cmd[LOADER_WRITE_SIZE+3] = {0};
	int ret;

	byte_cmd[0] = cmd;
	byte_cmd[1] = 0;//dummy

	switch (cmd) {
	case LOADER_CMD_FILE_INFO:
	case LOADER_CMD_SEND_DATA:
	case LOADER_CMD_RESEND_DATA:
		if (buf == NULL) {
			LED_ERR("buf is NULL\n");
			return -ENOMEM;
		}
		memcpy(&byte_cmd[2], buf, size);
		byte_cmd[LOADER_WRITE_SIZE+2] = checksum;
		break;
	case LOADER_CMD_ERASE_MCU_FLASH:
	case LOADER_CMD_EXIT_LOADER:
		break;
	default:
		LED_ERR("not supported command: %x\n", cmd);
		return -EINVAL;
	}

	if (LOADER_RAWDATA)
		LED_INFO("cmd: %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x\n",
				byte_cmd[0],  byte_cmd[1],  byte_cmd[2],  byte_cmd[3],  byte_cmd[4],  byte_cmd[5],
				byte_cmd[6],  byte_cmd[7],  byte_cmd[8],  byte_cmd[9],  byte_cmd[10], byte_cmd[11],
				byte_cmd[12], byte_cmd[13], byte_cmd[14], byte_cmd[15], byte_cmd[16], byte_cmd[17],
				byte_cmd[18]);

	ret = i2c_write(client, LOADER_WRITE_SIZE+3, byte_cmd);
	if (ret < 0) {
		LED_ERR("write 0x%02x loader command failed(%d)\n", cmd, ret);
		return ret;
	}

	return 0;
}

static int leds_get_info(struct i2c_client *client, u8 mode, u8 info_type, u8 len, u8 *buf)
{
	int ret;
	u8 wbuf[I2C_GET_INFO_PRE_LEN];

	if (mode == NORMAL_MODE) {
		wbuf[0] = NORMAL_CMD_GET_INFO;
		wbuf[1] = info_type;
		ret = i2c_write(client, I2C_GET_INFO_PRE_LEN, wbuf);
		if (ret < 0) {
			LED_ERR("i2c_write failed for NORMAL_CMD_GET_INFO(%d)\n", ret);
			goto failed;
		}

		ret = i2c_read(client, len, buf);
		if (ret < 0) {
			LED_ERR("i2c_read failed for NORMAL_CMD_GET_INFO(%d)\n", ret);
			goto failed;
		}
	} else if (mode == BOOTLOADER_MODE){
		wbuf[0] = LOADER_CMD_GET_INFO;
		wbuf[1] = info_type;
		ret = i2c_write(client, I2C_GET_INFO_PRE_LEN, wbuf);
		if (ret < 0) {
			LED_ERR("i2c_write failed for LOADER_CMD_GET_INFO(%d)\n", ret);
			goto failed;
		}

		ret = i2c_read(client, len, buf);
		if (ret < 0) {
			LED_ERR("i2c_read failed for LOADER_CMD_GET_INFO(%d)\n", ret);
			goto failed;
		}
	} else {
		LED_ERR("invalid mode: %d\n", mode);
		return -EINVAL;
	}

	return 0;

failed:
	return ret;
}

static void enter_bootloader(struct led_wheel_data *led_data)
{
	LED_INFO("enter bootloader\n");
	gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_disable);
	msleep(500);
	gpio_direction_output(LED_BOOT_PIN, led_data->pin_disable);
	msleep(20);
	gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_enable);
}

static void power_reset_delay(struct led_wheel_data *led_data, int delay)
{
	LED_INFO("%dms\n", delay);
	gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_disable);
	msleep(delay);
	gpio_direction_output(LED_BOOT_PIN, led_data->pin_enable);
	msleep(20);
	gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_enable);
}

static void deep_power_reset(struct led_wheel_data *led_data)
{
	power_reset_delay(led_data, 1000);
}

static void power_reset(struct led_wheel_data *led_data)
{
	power_reset_delay(led_data, 500);
}

static void i2c_reset(struct led_wheel_data *led_data)
{
	gpio_direction_output(LED_BOOT_PIN, led_data->pin_disable);
	msleep(50);
	gpio_direction_output(LED_BOOT_PIN, led_data->pin_enable);
}

static ssize_t raw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	int size_s, ret=0;

	if (led_state != IDLE_STATE) {
		LED_ERR("led is busy\n");
		return -EBUSY;
	}

	if(!is_hex(buf[size-1])) {
		size_s = size - 1;
	} else {
		size_s = size;
	}

	if (size_s != 14) {
		LED_ERR("size is not equal 14\n");
		return -EINVAL;
	}

	ret = send_raw_cmd(led_data, buf, size_s);
	if (ret < 0) {
		LED_ERR("send_raw_cmd failed, %d\n", ret);
	}

	return ret < 0 ? ret : size;
}

static ssize_t update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	char fw_name[255];
	int ret, retry=0;

	if (size >= 255) {
		LED_ERR("file name too long\n");
		return -EINVAL;
	}

	memcpy(fw_name, buf, size);
	if (fw_name[size - 1] == '\n')
		fw_name[size - 1] = '\0';
	else
		fw_name[size] = '\0';

	LED_INFO("fw_name = %s\n", fw_name);

	while (retry < RETRY_COUNT_UPDATE) {
		ret = start_fw_update(led_data, fw_name, 0);
		if (!ret)
			break;
		retry++;
		LED_ERR("update failed, try %d\n", retry);
	}
	return ret < 0 ? ret : size;
}

static ssize_t bootloader_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	long value = 0;

	if(!strict_strtol(buf, 0, &value))
		if (value)
			enter_bootloader(led_data);

	return size;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	long value = 0;

	if(!strict_strtol(buf, 0, &value))
		if (value)
			power_reset(led_data);

	return size;
}

static ssize_t power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	long value = 0;

	if(!strict_strtol(buf, 0, &value)) {
		if (value) {
			LED_INFO("power on\n");
			gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_enable);
		} else {
			LED_INFO("power off\n");
			gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_disable);
		}
	}

	return size;
}

static ssize_t boot_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	long value = 0;

	if(!strict_strtol(buf, 0, &value)) {
		if (value) {
			LED_INFO("boot on\n");
			gpio_direction_output(LED_BOOT_PIN, led_data->pin_enable);
		} else {
			LED_INFO("boot off\n");
			gpio_direction_output(LED_BOOT_PIN, led_data->pin_disable);
		}
	}

	return size;
}

static ssize_t leds_status_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	u8 color_buf[8][I2C_GET_INFO_DATA_LEN] = {{0}};
	u8 pattern_buf[8][I2C_GET_INFO_DATA_LEN] = {{0}};
	u8 timeout_err_buf[I2C_GET_INFO_DATA_LEN] = {0};
	int i, ret, size=0, len, timeout_err;

	for (i = 0; i < 8; i++) {
		ret = leds_get_info(led_data->client, NORMAL_MODE, INFO_TYPE_COLOR_BRIGHT_BASE+i,
			I2C_GET_INFO_DATA_LEN, color_buf[i]);
		if (ret < 0) {
			LED_ERR("get color and bright info failed(%d)\n", ret);
			return ret;
		}

		ret = leds_get_info(led_data->client, NORMAL_MODE, INFO_TYPE_BEHAVIOR_BASE+i,
			I2C_GET_INFO_DATA_LEN, pattern_buf[i]);
		if (ret < 0) {
			LED_ERR("get behavior info failed(%d)\n", ret);
			return ret;
		}
	}
	ret = leds_get_info(led_data->client, NORMAL_MODE, INFO_TYPE_TIMEOUT_ERR,
			I2C_GET_INFO_DATA_LEN, timeout_err_buf);
	if (ret < 0) {
		LED_ERR("get timeout error info failed(%d)\n", ret);
		return ret;
	}

	len = sprintf(buf, "wheel(%d):\n", led_data->wheel_num);
	buf += len;
	size += len;
	for (i = 0; i < 8; i++) {
		len = sprintf(buf, "LED-%d: R=%02x G=%02x B=%02x BRIGHT=%02x", i,
			color_buf[i][0], color_buf[i][1], color_buf[i][2], color_buf[i][3]);
		buf += len;
		size += len;

		len = sprintf(buf, " PATTERN=%02x\n", pattern_buf[i][0]);
		buf += len;
		size += len;
	}
	timeout_err = timeout_err_buf[0] | timeout_err_buf[1] << 8 |
			timeout_err_buf[2] << 16 | timeout_err_buf[3] << 24;
	len = sprintf(buf, "TIMEOUT_ERR=%d\n", timeout_err);
	buf += len;
	size += len;
	return size;
}

static ssize_t fw_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};
	int ret=0;

	if (led_state != IDLE_STATE) {
		LED_ERR("led is busy\n");
		return 0;
	}

	ret = get_app_ver(led_data, read_buf);
	if (ret < 0) {
		LED_ERR("get_app_ver failed(%d)\n", ret);
		return 0;
	}
	return sprintf(buf, "%d.%d\n", read_buf[0], read_buf[1]);
}

static ssize_t loader_fw_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};
	int ret;

	ret = get_bootloader_ver(led_data, read_buf);
	if (ret < 0) {
		LED_ERR("leds_get_info failed(%d)\n", ret);
		return 0;
	}
	return sprintf(buf, "%d.%d\n", read_buf[3], read_buf[4]);
}

static ssize_t block_checksum_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};
	int ret;

	ret = leds_get_info(led_data->client, BOOTLOADER_MODE, INFO_TYPE_BLOCK_CHECKSUM,
			I2C_GET_INFO_DATA_LEN, read_buf);
	if (ret < 0) {
		LED_ERR("leds_get_info failed(%d)\n", ret);
		return 0;
	}

	return sprintf(buf, "0x%x\n", read_buf[0] | (read_buf[1] << 8));
}

static ssize_t image_checksum_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct led_wheel_data *led_data = dev_get_drvdata(dev);
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};
	int ret;

	ret = leds_get_info(led_data->client, BOOTLOADER_MODE, INFO_TYPE_IMAGE_CHECKSUM,
			I2C_GET_INFO_DATA_LEN, read_buf);
	if (ret < 0) {
		LED_ERR("leds_get_info failed(%d)\n", ret);
		return 0;
	}

	return sprintf(buf, "0x%x\n", read_buf[0] | (read_buf[1] << 8));
}
static DEVICE_ATTR(rawdata, S_IWUSR | S_IWGRP, NULL, raw_store);
static DEVICE_ATTR(update, S_IWUSR | S_IWGRP, NULL, update_store);
static DEVICE_ATTR(bootloader, S_IWUSR | S_IWGRP, NULL, bootloader_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, reset_store);
static DEVICE_ATTR(power_en, S_IWUSR | S_IWGRP, NULL, power_store);
static DEVICE_ATTR(boot_pin, S_IWUSR | S_IWGRP, NULL, boot_store);
static DEVICE_ATTR(leds_status, S_IRUGO, leds_status_show, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_version_show, NULL);
static DEVICE_ATTR(loader_fw_ver, S_IRUGO, loader_fw_version_show, NULL);
static DEVICE_ATTR(loader_block_checksum, S_IRUGO, block_checksum_show, NULL);
static DEVICE_ATTR(loader_image_checksum, S_IRUGO, image_checksum_show, NULL);

static struct attribute *leds_wheel_sysfs_entries[] = {
	&dev_attr_rawdata.attr,
	&dev_attr_update.attr,
	&dev_attr_bootloader.attr,
	&dev_attr_reset.attr,
	&dev_attr_power_en.attr,
	&dev_attr_boot_pin.attr,
	&dev_attr_leds_status.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_loader_fw_ver.attr,
	&dev_attr_loader_block_checksum.attr,
	&dev_attr_loader_image_checksum.attr,
	NULL,
};

static const struct attribute_group leds_wheel_sysfs_group = {
	.attrs = leds_wheel_sysfs_entries,
};

static int get_bootloader_ver(struct led_wheel_data *led_data, u8 *buf)
{
	int ret=0, count=0;
	while (count < RETRY_COUNT_GETINFO) {
		ret = leds_get_info(led_data->client, BOOTLOADER_MODE,
				INFO_TYPE_IMAGE_CHECKSUM, I2C_GET_INFO_DATA_LEN, buf);
		if (ret == 0)
			break;
		count++;
		msleep(50);
	}
	return ret;
}

static int check_bootloader(struct led_wheel_data *led_data)
{
	int ret=0, retry=0;
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};

	while (retry < RETRY_COUNT_CHECK) {
		ret = get_bootloader_ver(led_data, read_buf);
		if (ret == 0)
			break;
		retry++;
		LED_ERR("get_bootloader_ver failed(%d), try %d\n", ret, retry);
		enter_bootloader(led_data);
		msleep(100);
	}
	if (ret < 0)
		return ret;

	LED_INFO("wheel(%d) loader fw version = %d.%d\n", led_data->wheel_num, read_buf[3], read_buf[4])

	if (read_buf[3] != 0xFF && read_buf[4] != 0xFF) {
		return 1;
	}

	return 0;
}

static int get_app_ver(struct led_wheel_data *led_data, u8 *buf)
{
	int ret=0, count=0;
	while (count < RETRY_COUNT_GETINFO) {
		ret = leds_get_info(led_data->client, NORMAL_MODE,
				INFO_TYPE_FW_VERSION, I2C_GET_INFO_DATA_LEN, buf);
		if (ret == 0)
			break;
		count++;
		msleep(50);
	}
	return ret;
}

static int check_app(struct led_wheel_data *led_data)
{
	int ret=0, retry=0;
	u8 read_buf[I2C_GET_INFO_DATA_LEN] = {0};

	while (retry < RETRY_COUNT_CHECK) {
		ret = get_app_ver(led_data, read_buf);
		if (ret == 0)
			break;
		retry++;
		LED_ERR("get_app_ver failed(%d), try %d\n", ret, retry);
		power_reset(led_data);
		msleep(100);
	}
	if (ret < 0)
		return ret;

	if (read_buf[0] == 0xFF || read_buf[1] == 0xFF) {
		LED_INFO("wheel(%d) is in bootloader\n", led_data->wheel_num);
		return -EINVAL;
	}

	if (read_buf[0] == 0) {
		LED_INFO("wheel(%d) fw version is invalid\n", led_data->wheel_num);
		return -EINVAL;
	}

	LED_INFO("wheel(%d) app fw version = %d.%d\n", led_data->wheel_num, read_buf[0], read_buf[1]);
	led_data->app_fw_ver_formal[led_data->wheel_num] = read_buf[0];
	led_data->app_fw_ver_beta[led_data->wheel_num] = read_buf[1];

	return 0;
}

static int is_fw_update(struct led_wheel_data *led_data)
{
	if (led_data->app_fw_ver_formal[led_data->wheel_num] == 0 &&
			led_data->app_fw_ver_beta[led_data->wheel_num] == 0) {
		LED_ERR("wheel(%d) ap fw version is not gotten\n", led_data->wheel_num);
		return 1;
	}

	if (led_data->app_fw_ver_formal[led_data->wheel_num] != LATEST_FW_VERSION_FORMAL ||
		led_data->app_fw_ver_beta[led_data->wheel_num] != LATEST_FW_VERSION_BETA) {
		LED_INFO("wheel(%d) ap fw needs to update(%d.%d -> %d.%d)\n",
				led_data->wheel_num,
				led_data->app_fw_ver_formal[led_data->wheel_num],
				led_data->app_fw_ver_beta[led_data->wheel_num],
				LATEST_FW_VERSION_FORMAL,
				LATEST_FW_VERSION_BETA);
		return 1;
	}
	LED_INFO("wheel(%d) ap fw is the latest\n", led_data->wheel_num);
	return 0;
}

static int loader_get_checksum(struct led_wheel_data *led_data, int type)
{
	u8 rbuf[I2C_GET_INFO_DATA_LEN] = {0};
	u16 checksum;
	int ret, retry=0;

	while (retry < RETRY_COUNT_GETINFO) {
		ret = leds_get_info(led_data->client, BOOTLOADER_MODE, type,
				I2C_GET_INFO_DATA_LEN, rbuf);
		if (ret < 0) {
			retry++;
			LED_ERR("leds_get_info failed(%d), try %d\n", ret, retry);
			msleep(50);
		} else {
			checksum = rbuf[0] | (rbuf[1] << 8);
			return checksum;
		}
	}
	return ret;
}

static int process_fw_update(struct led_wheel_data *led_data, const struct firmware *fw)
{
	u8 loader_checksum, cal_checksum;
	u16 total_loader_checksum, total_cal_checksum;
	int ret=0, i=0, wlen, block_size=LOADER_WRITE_SIZE, resend_count=0;
	const u8 *wbuf;

	LED_INFO("start\n");

	wbuf = fw->data;
	wlen = fw->size;

	while (i < wlen) {
		cal_checksum = (u8)calculate_checksum(&wbuf[i], block_size);
		ret = leds_loader_write(led_data->client, LOADER_CMD_SEND_DATA,
				&wbuf[i], block_size, cal_checksum);
		if (ret < 0) {
			LED_ERR("send data failed(%d)\n", ret);
			goto update_failed;
		}

resended:
		if (resend_count >= RETRY_COUNT_RESEND) {
			LED_ERR("resend_count >= %d, exit\n", RETRY_COUNT_RESEND);
			goto update_failed;
		}

		ret = loader_get_checksum(led_data, INFO_TYPE_BLOCK_CHECKSUM);
		if (ret < 0) {
			LED_ERR("get block checksum failed\n");
			goto update_failed;
		}
		loader_checksum = (u8)ret;

		//compare loader checksum and calculated checksum
		if (loader_checksum != cal_checksum) {
			LED_ERR("checksum is not matched(%x,%x), resend data\n", loader_checksum, cal_checksum);
			ret = leds_loader_write(led_data->client, LOADER_CMD_RESEND_DATA,
					&wbuf[i], block_size, cal_checksum);
			if (ret < 0) {
				LED_ERR("resend data failed(%d)\n", ret);
				goto update_failed;
			}
			resend_count++;
			goto resended;
		}

		i += LOADER_WRITE_SIZE;
		if (resend_count)
			resend_count = 0;

		//final block
		if (wlen > i && i > (wlen - LOADER_WRITE_SIZE)) {
			block_size = wlen - i;
			LED_INFO("final block size = %d\n", block_size);
		}
	}

	total_cal_checksum = calculate_checksum(wbuf, wlen);

	ret = loader_get_checksum(led_data, INFO_TYPE_IMAGE_CHECKSUM);
	if (ret < 0) {
		LED_ERR("get image checksum failed\n");
		goto update_failed;
	}
	total_loader_checksum = (u16)ret - 0x200;//remove debug space

	LED_INFO("total_cal_checksum=0x%04x, total_loader_checksum=0x%04x\n",
			total_cal_checksum, total_loader_checksum);

	if (total_loader_checksum != total_loader_checksum) {
		LED_ERR("compare image checksum failed\n");
		goto update_failed;
	}
	ret = 0;

update_failed:
	LED_INFO("end");
	return ret;
}

static int start_fw_update(struct led_wheel_data *led_data, char *fw_name, int check_cs)
{
	u16 image_checksum;
	int ret;
	const struct firmware *fw;

	ret = mutex_lock_interruptible(&led_data->lock);
	if (ret)
		return ret;

	led_state = UPDATTING_STATE;

	LED_INFO("update wheel(%d)\n", led_data->wheel_num);

	ret = request_firmware(&fw, fw_name, &led_data->client->dev);
	if (ret) {
		LED_ERR("failed to load firmware image: %s, (%d)\n", fw_name, ret);
		goto request_fw_failed;
	}
	LED_INFO("fw name=%s, size=%zu\n", fw_name, fw->size);

	if (check_cs == 1) {
		image_checksum = calculate_checksum(fw->data, fw->size);
		if (image_checksum != LATEST_FW_CHECKSUM) {
			LED_ERR("fw checksum is not valid, 0x%04x\n", image_checksum);
			ret = -EINVAL;
			goto checksum_failed;
		}
	}

	enter_bootloader(led_data);
	msleep(200);

	if(check_bootloader(led_data) <= 0) {
		LED_ERR("no bootloader mode or cmds failed, leave\n");
		ret = -EIO;
		goto update_failed;
	}

	ret = leds_loader_write(led_data->client, LOADER_CMD_ERASE_MCU_FLASH, NULL, 0, 0);
	if (ret < 0) {
		LED_ERR("failed to erase mcu flash(%d)\n", ret);
		goto update_failed;
	}
	LED_INFO("erase mcu flash successfully\n");

	ret = process_fw_update(led_data, fw);
	if (!ret) {
		LED_INFO("fw update successfully\n");
	} else {
		LED_INFO("fw update failed(%d)\n", ret);
	}

update_failed:
	power_reset(led_data);
	msleep(100);
checksum_failed:
	release_firmware(fw);
request_fw_failed:
	led_state = IDLE_STATE;
	mutex_unlock(&led_data->lock);
	return ret;
}

static void leds_update_work_func(struct work_struct *work)
{
	struct led_wheel_data *led_data;
	int ret, retry=0;

	led_data = container_of(work, struct led_wheel_data, update_work.work);

	if (is_fw_update(led_data)) {
		LED_INFO("try to update wheel(%d) fw\n", led_data->wheel_num);
		while (retry < RETRY_COUNT_UPDATE) {
			ret = start_fw_update(led_data, FW_NAME, 1);
			if (!ret) {
				ret = check_app(led_data);
				if (!ret)
					break;
			}
			retry++;
			LED_ERR("update failed, try %d\n", retry);
		}
	}
}

static void leds_cmd_work_func(struct work_struct *work)
{
	struct led_wheel_data *led_data = container_of(work, struct led_wheel_data, cmd_work.work);
	bool cos_mode;

#ifndef FACTORY_IMAGE
	cos_mode = check_cos_mode();
	if (cos_mode)
		return;

	LED_INFO("wheel(%d) issue bring-up commands\n", led_data->wheel_num);

	if (led_data->wheel_num == 0)
		send_raw_cmd(led_data, "15000000020FFF", 14);
	else if (led_data->wheel_num == 1)
		send_raw_cmd(led_data, "15000000020F01", 14);
#endif
}

static int leds_init_fs(struct led_wheel_data *led_data)
{
	int ret = 0;

	led_data->dev = device_create(leds_wheel_class, &led_data->client->dev, 0, led_data, "leds%d", tmp_wheel_num);
	if (IS_ERR(led_data->dev)) {
		ret = PTR_ERR(led_data->dev);
		goto device_failed;
	}

	ret = sysfs_create_group(&led_data->dev->kobj, &leds_wheel_sysfs_group);
	if (ret) {
		LED_ERR("failed to create sysfs interfaces(%d)\n", ret);
		goto sysfs_failed;
	}

	return 0;

sysfs_failed:
	device_unregister(led_data->dev);
device_failed:
	return ret;
}

static void leds_remove_fs(struct led_wheel_data *led_data)
{
	sysfs_remove_group(&led_data->dev->kobj, &leds_wheel_sysfs_group);
	device_unregister(led_data->dev);
}

static int leds_init_gpio(struct led_wheel_data *led_data)
{
	int ret;

	ret = gpio_request(LED_BOOT_PIN, "led_boot_pin");
	if (ret < 0) {
		LED_ERR("LED_BOOT_PIN request failed: %d\n", ret);
		return ret;
	}
	gpio_direction_output(LED_BOOT_PIN, led_data->pin_enable);

	ret = gpio_request(LED_POWER_EN_PIN, "led_power_en");
	if (ret < 0) {
		LED_ERR("LED_POWER_EN_PIN request failed: %d\n", ret);
		return ret;
	}
	gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_enable);

	return 0;
}

static void leds_wheel_shutdown(struct i2c_client *client)
{
	struct led_wheel_data *led_data = i2c_get_clientdata(client);

	if (gpio_get_value(LED_POWER_EN_PIN) != led_data->pin_disable) {
		LED_INFO("power off leds\n");
		gpio_direction_output(LED_POWER_EN_PIN, led_data->pin_disable);
	}
}

static int leds_wheel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct led_wheel_data *led_data;
	int ret, hw_id;

	LED_INFO("I2C Address: 0x%02x\n", client->addr);

	led_data = kzalloc(sizeof(struct led_wheel_data), GFP_KERNEL);
	if (led_data == NULL) {
		LED_ERR("no memory for device\n");
		ret = -ENOMEM;
		goto alloc_failed;
	}

	hw_id = asustek_get_hw_rev();
	LED_INFO("hw id = %d\n", hw_id);
	if (hw_id >= HW_ID_PR) {
		led_data->pin_enable = 0;
		led_data->pin_disable = 1;
	} else {
		led_data->pin_enable = 1;
		led_data->pin_disable = 0;
	}

	LED_INFO("pin_enable = %d, pin_disable = %d\n",
			led_data->pin_enable, led_data->pin_disable);

	led_data->client = client;
	led_data->wheel_num = tmp_wheel_num;
	i2c_set_clientdata(client, led_data);

	mutex_init(&led_data->lock);

	//first entry
	if (!tmp_wheel_num) {
		ret = leds_init_gpio(led_data);
		if (ret < 0) {
			LED_ERR("gpio init failed\n");
			goto probe_failed;
		}
		msleep(50);
		leds_wq = create_singlethread_workqueue("leds_wq");
	}

	// check app version on ER and later boards.
	if (hw_id >= HW_ID_ER) {
		ret = check_app(led_data);
		if (ret < 0) {
			LED_ERR("check wheel(%d) app failed(%d)\n", tmp_wheel_num, ret);
		}
	}

	ret = leds_init_fs(led_data);
	if (ret < 0) {
		LED_ERR("file system init failed\n");
		goto probe_failed;
	}

	if (hw_id >= HW_ID_ER) {
		INIT_DELAYED_WORK(&led_data->update_work, leds_update_work_func);
		queue_delayed_work(leds_wq, &led_data->update_work, 0.2*HZ);

		INIT_DELAYED_WORK(&led_data->cmd_work, leds_cmd_work_func);
		queue_delayed_work(leds_wq, &led_data->cmd_work, 0.5*HZ);
	}

	tmp_wheel_num++;
	return 0;

probe_failed:
	kfree(led_data);
alloc_failed:
	tmp_wheel_num++;
	return ret;
}

static int leds_wheel_remove(struct i2c_client *client)
{
	struct led_wheel_data *led_data = i2c_get_clientdata(client);

	LED_INFO("\n");

	if (asustek_get_hw_rev() >= HW_ID_ER) {
		cancel_delayed_work_sync(&led_data->update_work);
		cancel_delayed_work_sync(&led_data->cmd_work);
	}

	if (!led_data->wheel_num)
		destroy_workqueue(leds_wq);

	leds_remove_fs(led_data);
	kfree(led_data);

	return 0;
}

static const struct i2c_device_id leds_wheel_id[] = {
	{ "leds_wheel", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, leds_wheel_id);

static const struct acpi_device_id leds_wheel_acpi_table[] = {
	{ "LWD0000", 0 },
	{ "LWD0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, leds_wheel_acpi_id);

static struct i2c_driver leds_wheel_driver = {
	.probe	  = leds_wheel_probe,
	.remove	 = leds_wheel_remove,
	.shutdown = leds_wheel_shutdown,
	.id_table   = leds_wheel_id,
	.driver = {
		.name	 = "leds_wheel",
		.owner	= THIS_MODULE,
		.acpi_match_table = leds_wheel_acpi_table,
	},
};

static int leds_wheel_init(void)
{
	int ret;

	LED_DBG("\n");

	tmp_wheel_num = 0;
	leds_wheel_class = class_create(THIS_MODULE, "leds_wheel");
	if (IS_ERR(leds_wheel_class)) {
		LED_ERR("create leds_wheel class failed\n");
		return PTR_ERR(leds_wheel_class);
	}

	ret = i2c_add_driver(&leds_wheel_driver);
	return ret;
}

static void __exit leds_wheel_exit(void)
{
	LED_DBG("\n");
	class_destroy(leds_wheel_class);
	i2c_del_driver(&leds_wheel_driver);
}

rootfs_initcall(leds_wheel_init);
module_exit(leds_wheel_exit);

MODULE_DESCRIPTION("LEDs Wheel Driver");
MODULE_LICENSE("GPL");
