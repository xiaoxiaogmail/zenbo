#ifndef _LEDS_WHEEL_H_
#define _LEDS_WHEEL_H_

#define LEDS_TAG "LEDS-WHEEL"

//Flags
#define LOADER_RAWDATA 0

#define LED_INFO(fmt,arg...) pr_info("%s: %s: "fmt, LEDS_TAG, __func__, ##arg);

#ifdef FACTORY_IMAGE
#define LED_DBG(fmt,arg...) pr_info("%s: %s: "fmt, LEDS_TAG, __func__, ##arg);
#else
#define LED_DBG(fmt,arg...) pr_debug("%s: %s: "fmt, LEDS_TAG, __func__, ##arg);
#endif

#define LED_ERR(fmt,arg...) pr_err("%s: %s: "fmt, LEDS_TAG, __func__, ##arg);

#define HW_ID_ER 2
#define HW_ID_PR 3

#define LED_POWER_EN_PIN 292
#define LED_BOOT_PIN 404

#define NORMAL_CMD_GET_INFO 0x78

#define LOADER_CMD_FILE_INFO 0x80
#define LOADER_CMD_SEND_DATA 0x81
#define LOADER_CMD_RESEND_DATA 0x82
#define LOADER_CMD_ERASE_MCU_FLASH 0x86
#define LOADER_CMD_EXIT_LOADER 0x87
#define LOADER_CMD_GET_INFO 0x88

#define NORMAL_WRITE_SIZE 7
#define NORMAL_READ_SIZE 7
#define LOADER_WRITE_SIZE 16
#define LOADER_READ_SIZE 7

#define NORMAL_MODE 0
#define BOOTLOADER_MODE 1

#define IDLE_STATE 0
#define UPDATTING_STATE 1

#define INFO_TYPE_FW_VERSION 0
#define INFO_TYPE_BLOCK_CHECKSUM 0
#define INFO_TYPE_IMAGE_CHECKSUM 1
#define INFO_TYPE_COLOR_BRIGHT_BASE 0x10
#define INFO_TYPE_BEHAVIOR_BASE 0x20
#define INFO_TYPE_TIMEOUT_ERR 0x30

#define I2C_GET_INFO_PRE_LEN 2
#define I2C_GET_INFO_DATA_LEN 5

#define RETRY_COUNT_SENDCMD 3
#define RETRY_COUNT_GETINFO 3
#define RETRY_COUNT_CHECK 3
#define RETRY_COUNT_RESEND 5
#define RETRY_COUNT_UPDATE 3

#define ERROR_CODE_I2C_RECOVERY 998;
#define ERROR_CODE_POWER_RECOVERY 999;

#define FW_NAME "led_fw"
#define LATEST_FW_VERSION_FORMAL 1
#define LATEST_FW_VERSION_BETA 14
#define LATEST_FW_CHECKSUM 0x129e

struct led_wheel_data {
	struct device *dev;
	struct i2c_client *client;
	struct delayed_work update_work;
	struct delayed_work cmd_work;
	struct mutex lock;

	u8 wheel_num;
	u8 app_fw_ver_formal[2];
	u8 app_fw_ver_beta[2];
	u8 pin_enable;
	u8 pin_disable;
};

#endif /* _LEDS_WHEEL_H_ */

