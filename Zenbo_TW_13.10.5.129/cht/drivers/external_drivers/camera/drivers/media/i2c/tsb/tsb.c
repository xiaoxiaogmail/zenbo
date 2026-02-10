/*
 * Support for Toshiba T4K37(13MP)/T4K35(8MP) camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <asm/intel-mid.h>
#include <linux/atomisp_platform.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/acpi.h>
#include <linux/atomisp_gmin_platform.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include "tsb.h"

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#define CHT_MCLK_WORKAROUND 1
#define DEBUG_TEMP 1
#define T4K37_RESOLUTION "13M"
#define T4K37_MODULE "T4K37"
#define TSB_BINNING_MODE 1


static u8 tsb_otp_data[32];
static struct tsb_otp_struct tsb_otp;
static unsigned int ATD_t4k37_status;
static int exposure_return0;
static unsigned int read_register;
static struct i2c_client *g_client;
static unsigned int g_vcm_pos;
#ifdef TSB_BINNING_MODE
static unsigned int BINNING_SUM;
#endif
static struct tsb_device *g_dev;
static unsigned int power_count = 0;

static enum atomisp_bayer_order tsb_bayer_order_mapping[] = {
#if 0
	atomisp_bayer_order_rggb,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_gbrg,
	atomisp_bayer_order_bggr
#else
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg
#endif
};

static int
tsb_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[TSB_SHORT_MAX];
	int err, i;
	int retry = 0;

	if (len > TSB_BYTE_MAX) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));
	memset(data, 0 , sizeof(data));

again:

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);

	/* high byte comes first */
	if (len == TSB_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	if (err == 2)
		return 0;

	dev_err(&client->dev, "i2c read from offset 0x%x error %d", reg, err);
	if (retry < I2C_RETRY_COUNT) {
		retry++;
		msleep(20);
		goto again;
	}

	return err == 2 ? 0 : -EIO;
}


static int tsb_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret == num_msg)
		return 0;

	dev_err(&client->dev, "i2c write error %d", ret);
	if (retry < I2C_RETRY_COUNT) {
		retry++;
		msleep(20);
		goto again;
	}

	return ret == num_msg ? 0 : -EIO;
}

int
tsb_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != TSB_8BIT && data_length != TSB_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == TSB_8BIT)
		data[2] = (u8)(val);
	else {
		/* TSB_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = tsb_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * tsb_write_reg_array - Initializes a list of tsb registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __tsb_flush_reg_array, __tsb_buf_reg_array() and
 * __tsb_write_reg_is_consecutive() are internal functions to
 * tsb_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __tsb_flush_reg_array(struct i2c_client *client,
				     struct tsb_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return tsb_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __tsb_buf_reg_array(struct i2c_client *client,
				   struct tsb_write_ctrl *ctrl,
				   const struct tsb_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case TSB_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case TSB_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= TSB_MAX_WRITE_BUF_SIZE)
		return __tsb_flush_reg_array(client, ctrl);

	return 0;
}

static int
__tsb_write_reg_is_consecutive(struct i2c_client *client,
				   struct tsb_write_ctrl *ctrl,
				   const struct tsb_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->sreg;
}

static int tsb_write_reg_array(struct i2c_client *client,
				   const struct tsb_reg *reglist)
{
	const struct tsb_reg *next = reglist;
	struct tsb_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != TSB_TOK_TERM; next++) {
		switch (next->type & TSB_TOK_MASK) {
		case TSB_TOK_DELAY:
			err = __tsb_flush_reg_array(client, &ctrl);
			if (err) {
				v4l2_err(client, "%s: token delay, flush write error, aborted\n",
					 __func__);
				return err;
			}
			msleep(next->val);
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__tsb_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __tsb_flush_reg_array(client, &ctrl);
				if (err) {
					v4l2_err(client, "%s: no consecutive, flush write error, aborted, err=%d\n",
						 __func__, err);
					return err;
				}
			}
			err = __tsb_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __tsb_flush_reg_array(client, &ctrl);
}

static int __tsb_min_fps_diff(int fps, const struct tsb_fps_setting *fps_list)
{
	int diff = INT_MAX;
	int i;

	if (fps == 0)
		return 0;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps) < diff)
			diff = abs(fps_list[i].fps - fps);
	}

	return diff;
}

static int __tsb_nearest_fps_index(int fps,
					const struct tsb_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps)
		    < abs(fps_list[fps_index].fps - fps))
			fps_index = i;
	}
	return fps_index;
}

/*
 * This is to choose the nearest fps setting above the requested fps
 * fps_list should be in ascendant order.
 */
static int __tsb_above_nearest_fps_index(int fps,
					const struct tsb_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (fps <= fps_list[i].fps) {
			fps_index = i;
			break;
		}
	}

	return fps_index;
}

static int __tsb_get_max_fps_index(
				const struct tsb_fps_setting *fps_settings)
{
	int i;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (fps_settings[i].fps == 0)
			break;
	}

	return i - 1;
}

static int __tsb_update_exposure_timing(struct i2c_client *client, u16 exposure,
			u16 llp, u16 fll)
{
	int ret = 0;

	/* Increase the VTS to match exposure + margin */
	if (exposure > fll - TSB_INTEGRATION_TIME_MARGIN)
		fll = exposure + TSB_INTEGRATION_TIME_MARGIN;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_LINE_LENGTH_PIXELS, llp);
	if (ret)
		return ret;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_FRAME_LENGTH_LINES, fll);
	if (ret)
		return ret;

	if (exposure)
		ret = tsb_write_reg(client, TSB_16BIT,
			TSB_COARSE_INTEGRATION_TIME, exposure);
	return ret;
}

static int __tsb_update_gain(struct v4l2_subdev *sd, u16 gain)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/* set global gain */
	/* t4k35 analog gain: x1~x16, 0x0028~0x0280, AG[15:0] = 0x0028 * gain */
	/* t4k37 analog gain: x1~x16, 0x0037~0x0370, AG[15:0] = 0x0037 * gain */
	ret = tsb_write_reg(client, TSB_16BIT, TSB_GLOBAL_GAIN, gain);
	if (ret)
		return ret;

	return ret;
}

static int __tsb_update_digital_gain(struct i2c_client *client, u16 digitgain)
{
	struct tsb_write_buffer digit_gain;

	/* set digital gain */
	/* digital gain: x1~x3.99, 0x0100~0x03FF */
	digit_gain.addr = cpu_to_be16(TSB_DGC_ADJ);
	digit_gain.data[0] = (digitgain >> 8) & 0xFF;
	digit_gain.data[1] = digitgain & 0xFF;
	digit_gain.data[2] = (digitgain >> 8) & 0xFF;
	digit_gain.data[3] = digitgain & 0xFF;
	digit_gain.data[4] = (digitgain >> 8) & 0xFF;
	digit_gain.data[5] = digitgain & 0xFF;
	digit_gain.data[6] = (digitgain >> 8) & 0xFF;
	digit_gain.data[7] = digitgain & 0xFF;

	return tsb_i2c_write(client, TSB_DGC_LEN, (u8 *)&digit_gain);
}

static int tsb_set_exposure_gain(struct v4l2_subdev *sd, u16 coarse_itg,
	u16 gain, u16 digitgain)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	pr_debug("%s, coarse_time %d Again %d Dgain %d\n", __func__, coarse_itg, gain, digitgain);

	if (exposure_return0)
		return 0;

	/* Validate exposure:  cannot exceed VTS-6 where VTS is 16bit */
	coarse_itg = clamp_t(u16, coarse_itg, 0, TSB_MAX_EXPOSURE_SUPPORTED);

	/* Validate gain: must not exceed maximum 12bit value */
	gain = clamp_t(u16, gain, 55, TSB_MAX_GLOBAL_GAIN_SUPPORTED);

	/* Validate digital gain: must not exceed 10 bit value*/
	digitgain = clamp_t(u16, digitgain, 256, TSB_MAX_DIGITAL_GAIN_SUPPORTED);

	mutex_lock(&dev->input_lock);

	/* GROUPED_PARAMETER_HOLD_ENABLE */
	ret = tsb_write_reg_array(client, tsb_param_hold);
	if (ret)
		pr_info("%s: group hold fail\n", __func__);

	ret = __tsb_update_exposure_timing(client, coarse_itg,
			dev->pixels_per_line, dev->lines_per_frame);
	if (ret) {
		pr_info("%s: update exposure fail\n", __func__);
		goto out;
	}
	dev->coarse_itg = coarse_itg;

	ret = __tsb_update_gain(sd, gain);
	if (ret) {
		pr_info("%s: update analog gain fail\n", __func__);
		goto out;
	}
	dev->gain = gain;

	ret = __tsb_update_digital_gain(client, digitgain);
	if (ret) {
		pr_info("%s: update digital gain fail\n", __func__);
		goto out;
	}
	dev->digital_gain = digitgain;

out:
	/* GROUPED_PARAMETER_HOLD_DISABLE */
	ret = tsb_write_reg_array(client, tsb_param_update);
	if (ret)
		pr_info("%s: group update fail\n", __func__);

	mutex_unlock(&dev->input_lock);
	return ret;
}

static long tsb_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	return tsb_set_exposure_gain(sd, exposure->integration_time[0],
				exposure->gain[0], exposure->gain[1]);
}


static int __tsb_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);

	pr_info("%s\n", __func__);
	if (dev->sensor_id == TSB_ID_DEFAULT) {
		pr_info("%s, sensor id = %u\n", __func__, dev->sensor_id);
		return 0;
	}

	/* Sets the default FPS */
	dev->fps_index = 0;
	dev->curr_res_table = dev->mode_tables->res_preview;
	dev->entries_curr_table = dev->mode_tables->n_res_preview;

	return tsb_write_reg_array(client,
			dev->mode_tables->init_settings);
}

static int tsb_init(struct v4l2_subdev *sd, u32 val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);
	ret = __tsb_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long tsb_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return tsb_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int power_ctrl(struct v4l2_subdev *sd, bool flag)
{
	int ret;
	struct tsb_device *dev = to_tsb_sensor(sd);

	pr_debug("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (!dev || !dev->platform_data)
		return -ENODEV;

	/* Non-gmin platforms use the legacy callback */
	if (dev->platform_data->power_ctrl)
		return dev->platform_data->power_ctrl(sd, flag);

	/* This driver assumes "internal DVDD, PWDNB tied to DOVDD".
	 * In this set up only gpio0 (XSHUTDN) should be available
	 * but in some products (for example ECS) gpio1 (PWDNB) is
	 * also available. If gpio1 is available we emulate it being
	 * tied to DOVDD here. */
	if (flag) {
		dev->platform_data->gpio0_ctrl(sd, 0);
		usleep_range(4000, 4100);

		/* vcm power */
		ret = dev->platform_data->v2p8_vcm_ctrl(sd, 1);
		if (ret)
			pr_err("%s power on 2v8_vcm failed\n", __func__);

		ret = dev->platform_data->v1p8_ctrl(sd, 1);
		if (ret) {
			pr_err("%s power on 1v8 failed\n", __func__);
			goto v1p8fail;
		}
		ret = dev->platform_data->v2p8_ctrl(sd, 1);
		if (ret) {
			pr_err("%s power on 2v8 failed\n", __func__);
			goto v2p8fail;
		}
		ret = dev->platform_data->v1p2_ctrl(sd, 1);
		if (ret) {
			pr_err("%s power on 1v2 failed\n", __func__);
			goto v1p2fail;
		}
		usleep_range(2000, 2100);
	} else {
		ret = dev->platform_data->v1p2_ctrl(sd, 0);
		if (ret)
			pr_err("%s power off 1v2 failed\n", __func__);

		ret = dev->platform_data->v2p8_ctrl(sd, 0);
		if (ret)
			pr_err("%s power off 2v8 failed\n", __func__);

		ret |= dev->platform_data->v1p8_ctrl(sd, 0);
		if (ret)
			pr_err("%s power off 1v8 failed\n", __func__);

		/* vcm power */
		ret |= dev->platform_data->v2p8_vcm_ctrl(sd, 0);
		if (ret)
			pr_err("%s power off 2v8_vcm failed\n", __func__);
	}

	pr_debug("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;

v1p2fail:
	ret = dev->platform_data->v2p8_ctrl(sd, 0);
v2p8fail:
	ret = dev->platform_data->v1p8_ctrl(sd, 0);
v1p8fail:
	pr_err("%s() %s-- error\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int gpio_ctrl(struct v4l2_subdev *sd, bool flag)
{
	int ret;
	struct tsb_device *dev = to_tsb_sensor(sd);

	pr_debug("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (!dev || !dev->platform_data)
		return -ENODEV;

	/* Non-gmin platforms use the legacy callback */
	if (dev->platform_data->gpio_ctrl)
		return dev->platform_data->gpio_ctrl(sd, flag);

	ret = dev->platform_data->gpio0_ctrl(sd, flag);
	usleep_range(8000, 8500);

	pr_debug("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));

	return ret;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	pr_info("tsb %s, power_count=%d\n", __func__, power_count);
	if(power_count++ != 0) return 0;
	pr_debug("tsb %s+\n", __func__);

       /* power control */
	ret = power_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "power control on failed\n");
		goto fail_power;
	}

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "clk on failed\n");
		goto fail_clk;
	}

#if CHT_MCLK_WORKAROUND
	usleep_range(4000, 5000);
	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "clk on failed\n");
		goto fail_clk;
	}
	usleep_range(4000, 5000);
#endif

	/* gpio ctrl */
	ret = gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "gpio on failed\n");
		goto fail_gpio;
	}

	pr_debug("tsb %s-\n", __func__);

	return 0;
fail_gpio:
	gpio_ctrl(sd, 0);
fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	pr_info("tsb %s, power_count=%d\n", __func__, power_count);
	if(power_count == 0) {
		return 0;
	} else if(--power_count != 0) {
		return 0;
	}
	pr_debug("tsb %s+\n", __func__);

	/* gpio ctrl */
	ret = gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio off failed\n");

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk off failed\n");

#if CHT_MCLK_WORKAROUND
	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk off failed\n");
#endif

	/* power control */
	ret = power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "power control off failed.\n");

	pr_debug("tsb %s-\n", __func__);

	return ret;
}

static int __tsb_s_power(struct v4l2_subdev *sd, int on)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret = 0;
	int r = 0;
	pr_info("%s, on=%d\n", __func__, on);

	if (on == 0) {
		ret = power_down(sd);
		if (dev->vcm_driver && dev->vcm_driver->power_down)
			r = dev->vcm_driver->power_down(sd);
		if (ret == 0)
			ret = r;
		dev->power = 0;
	} else {
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			__tsb_init(sd, 0);
		}
		if (dev->vcm_driver && dev->vcm_driver->power_up)
			ret = dev->vcm_driver->power_up(sd);
		if (ret)
			return ret;
	}

	return ret;
}

static int tsb_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct tsb_device *dev = to_tsb_sensor(sd);

	mutex_lock(&dev->input_lock);
	ret = __tsb_s_power(sd, on);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int tsb_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct tsb_reg *reglist)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tsb_device *dev = to_tsb_sensor(sd);
	u32 vt_pix_clk_div;
	u32 vt_sys_clk_div;
	u32 pre_pll_clk_div;
	u32 pll_multiplier;

	const int ext_clk_freq_hz = 19200000;
	struct atomisp_sensor_mode_data *buf = &info->data;
	int ret;
	u16 data[TSB_INTG_BUF_COUNT];

	u32 vt_pix_clk_freq_mhz;
	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 read_mode;
	u32 div;
	int offset;

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, TSB_INTG_BUF_COUNT * sizeof(u16));
	ret = tsb_read_reg(client, 1, TSB_VT_PIX_CLK_DIV, data);
	if (ret)
		return ret;
	vt_pix_clk_div = data[0];
	if (vt_pix_clk_div == 0)
		vt_pix_clk_div = 1;

	ret = tsb_read_reg(client, 1, TSB_VT_SYS_CLK_DIV, data);
	if (ret)
		return ret;
	vt_sys_clk_div = data[0];
	if (vt_sys_clk_div == 0)
		vt_sys_clk_div = 1;
	ret = tsb_read_reg(client, 1, TSB_PRE_PLL_CLK_DIV, data);
	if (ret)
		return ret;
	pre_pll_clk_div = data[0] + 1;
	ret = tsb_read_reg(client, 2, TSB_PLL_MULTIPLIER, data);
	if (ret)
		return ret;
	pll_multiplier = data[0];

	memset(data, 0, TSB_INTG_BUF_COUNT * sizeof(u16));
	coarse_integration_time_min = 1;
	coarse_integration_time_max_margin = 6;
	/* 2:1/4binning enabled, 1:1/2binning enabled, 0:disabled */
	ret = tsb_read_reg(client, 1, TSB_H_BINNING_ENABLE, data);
	if (ret)
		return ret;
	buf->binning_factor_x = data[0] & TSB_MASK_2BIT;
	buf->binning_factor_x *= 2;
	if (buf->binning_factor_x == 0)
		buf->binning_factor_x = 1;
	read_mode = data[0] & TSB_MASK_2BIT;

	ret = tsb_read_reg(client, 1, TSB_V_BINNING_ENABLE, data);
	if (ret)
		return ret;
	buf->binning_factor_y = data[0] & TSB_MASK_2BIT;
	buf->binning_factor_y *= 2;
	if (buf->binning_factor_y == 0)
		buf->binning_factor_y = 1;

	ret = tsb_read_reg(client, 2, TSB_HORIZONTAL_OUTPUT_SIZE_H, data);
	if (ret)
		return ret;
	buf->output_width = data[0];

	ret = tsb_read_reg(client, 2, TSB_VERTICAL_OUTPUT_SIZE_H, data);
	if (ret)
		return ret;
	buf->output_height = data[0];

	if ((buf->output_width * buf->binning_factor_x * 3) > (buf->output_height * buf->binning_factor_y * 4)) {
		buf->crop_horizontal_start = 0;
		buf->crop_horizontal_end = buf->output_width * buf->binning_factor_x - 1;
		offset = (buf->output_width * buf->binning_factor_x*3/4 - buf->output_height * buf->binning_factor_y)/2;
		if (offset % 2)
			offset -= 1;
		buf->crop_vertical_start = offset;
		buf->crop_vertical_end = buf->output_height * buf->binning_factor_y + offset - 1;
	} else {
		buf->crop_vertical_start = 0;
		buf->crop_vertical_end = buf->output_height * buf->binning_factor_y - 1;
		offset = (buf->output_height * buf->binning_factor_y*4/3 - buf->output_width * buf->binning_factor_x)/2;
		if (offset % 2)
			offset -= 1;
		buf->crop_horizontal_start = offset;
		buf->crop_horizontal_end = buf->output_width * buf->binning_factor_x + offset - 1;
	}

	div = pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div;
	if (div == 0)
		return -EINVAL;

	if (dev->sensor_id == T4K37_NAME_ID)
		vt_pix_clk_freq_mhz = 4 * ext_clk_freq_hz / div;
	else
		vt_pix_clk_freq_mhz = 2 * ext_clk_freq_hz / div;
	vt_pix_clk_freq_mhz *= pll_multiplier;

	dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;

	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	pr_info("%s, vt_pix_clk_freq_mhz=%d\n", __func__, buf->vt_pix_clk_freq_mhz);
	buf->coarse_integration_time_min = coarse_integration_time_min;
	buf->coarse_integration_time_max_margin =
				coarse_integration_time_max_margin;

	buf->fine_integration_time_min = TSB_FINE_INTG_TIME;
	buf->fine_integration_time_max_margin = TSB_FINE_INTG_TIME;
	buf->fine_integration_time_def = TSB_FINE_INTG_TIME;
	buf->frame_length_lines = dev->lines_per_frame;
	buf->line_length_pck = dev->pixels_per_line;
	buf->read_mode = read_mode;



	pr_info("@%s, binning_factor %dx%d, crop %d~%d x %d~%d, size %dx%d\n", __func__,
		buf->binning_factor_x, buf->binning_factor_y, buf->crop_horizontal_start,
		buf->crop_horizontal_end, buf->crop_vertical_start, buf->crop_vertical_end,
		buf->output_width, buf->output_height);
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int tsb_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = tsb_read_reg(client, TSB_16BIT,
			       TSB_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

static int tsb_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return tsb_write_reg(client, TSB_16BIT, TSB_TEST_PATTERN_MODE, value);
}

static enum v4l2_mbus_pixelcode
tsb_translate_bayer_order(enum atomisp_bayer_order code)
{
#if 0
	switch (code) {
	case atomisp_bayer_order_rggb:
		pr_info("[DEBUG] bayer = V4L2_MBUS_FMT_SRGGB10_1X10\n");
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	case atomisp_bayer_order_grbg:
		pr_info("[DEBUG] bayer = V4L2_MBUS_FMT_SGRBG10_1X10\n");
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	case atomisp_bayer_order_bggr:
		pr_info("[DEBUG] bayer = V4L2_MBUS_FMT_SBGGR10_1X10\n");
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	case atomisp_bayer_order_gbrg:
		pr_info("[DEBUG] bayer = V4L2_MBUS_FMT_SGBRG10_1X10\n");
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	}
#else
		return V4L2_MBUS_FMT_SGRBG10_1X10;
#endif
	return 0;
}

static int tsb_v_flip(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	pr_info("%s, value=%d\n", __func__, value);

	ret = tsb_write_reg_array(client, tsb_param_hold);
	if (ret)
		return ret;
	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= TSB_VFLIP_BIT;
	else
		val &= ~TSB_VFLIP_BIT;
	ret = tsb_write_reg(client, TSB_8BIT,
			TSB_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info) {
		val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
		tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
		dev->format.code = tsb_translate_bayer_order(
			tsb_info->raw_bayer_order);
	}

	return tsb_write_reg_array(client, tsb_param_update);
}

static int tsb_h_flip(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	pr_info("%s, value=%d\n", __func__, value);

	ret = tsb_write_reg_array(client, tsb_param_hold);
	if (ret)
		return ret;
	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= TSB_HFLIP_BIT;
	else
		val &= ~TSB_HFLIP_BIT;
	ret = tsb_write_reg(client, TSB_8BIT,
			TSB_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info) {
		val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
		tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
		dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
	}

	return tsb_write_reg_array(client, tsb_param_update);
}

static int tsb_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (TSB_FOCAL_LENGTH_NUM << 16) | TSB_FOCAL_LENGTH_DEM;
	return 0;
}

static int tsb_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for tsb*/
	*val = (TSB_F_NUMBER_DEFAULT_NUM << 16) | TSB_F_NUMBER_DEM;
	return 0;
}

static int tsb_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (TSB_F_NUMBER_DEFAULT_NUM << 24) |
		(TSB_F_NUMBER_DEM << 16) |
		(TSB_F_NUMBER_DEFAULT_NUM << 8) | TSB_F_NUMBER_DEM;
	return 0;
}

static int tsb_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int tsb_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_y;

	return 0;
}

int tsb_vcm_power_up(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->power_up)
		return dev->vcm_driver->power_up(sd);
	return 0;
}

int tsb_vcm_power_down(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->power_down)
		return dev->vcm_driver->power_down(sd);
	return 0;
}

int tsb_vcm_init(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->init)
		return dev->vcm_driver->init(sd);
	return 0;
}

int tsb_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_vcm)
		return dev->vcm_driver->t_focus_vcm(sd, val);
	return 0;
}

int tsb_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret = 0;
	if (dev->vcm_driver && dev->vcm_driver->t_focus_abs) {
		ret = dev->vcm_driver->t_focus_abs(sd, value);
		if (ret)
			pr_info("tsb_t_focus_abs() fail\n");
		else
			g_vcm_pos = value;
	}
	return ret;
}
int tsb_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_rel)
		return dev->vcm_driver->t_focus_rel(sd, value);
	return 0;
}

int tsb_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_status)
		return dev->vcm_driver->q_focus_status(sd, value);
	return 0;
}

int tsb_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_abs)
		return dev->vcm_driver->q_focus_abs(sd, value);
	return 0;
}

int tsb_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_slew)
		return dev->vcm_driver->t_vcm_slew(sd, value);
	return 0;
}

int tsb_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_timing)
		return dev->vcm_driver->t_vcm_timing(sd, value);
	return 0;
}

#ifdef TSB_BINNING_MODE
int tsb_s_binning_mode(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	pr_info("%s: set binning mode to %d", __func__, value);
	if (value == 1) {
		tsb_write_reg(client, TSB_8BIT, TSB_BINNING_TYPE, 0x1);
		BINNING_SUM = 1;
	} else {
		tsb_write_reg(client, TSB_8BIT, TSB_BINNING_TYPE, 0x0);
		BINNING_SUM = 0;
	}

	return 0;
}
#endif

struct tsb_control tsb_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = tsb_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_h_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = TSB_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_focus_abs,
		.query = tsb_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = TSB_MAX_FOCUS_NEG,
			.maximum = TSB_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = tsb_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = TSB_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = TSB_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = TSB_FOCAL_LENGTH_DEFAULT,
			.maximum = TSB_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = TSB_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = tsb_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = TSB_F_NUMBER_DEFAULT,
			.maximum = TSB_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = TSB_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = tsb_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = TSB_F_NUMBER_RANGE,
			.maximum =  TSB_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = TSB_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = tsb_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = TSB_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = tsb_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = TSB_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = tsb_g_bin_factor_y,
	},
#ifdef TSB_BINNING_MODE
	{
		.qc = {
			.id = V4L2_CID_BINNING_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "binning mode",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_s_binning_mode,
	},
#endif
};
#define N_CONTROLS (ARRAY_SIZE(tsb_controls))

static struct tsb_control *tsb_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (tsb_controls[i].qc.id == id)
			return &tsb_controls[i];
	return NULL;
}

static int tsb_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct tsb_control *ctrl = tsb_find_control(qc->id);
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* tsb control set/get */
static int tsb_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tsb_control *s_ctrl;
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = tsb_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int tsb_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tsb_control *octrl = tsb_find_control(ctrl->id);
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
static int distance(struct tsb_resolution const *res, u32 w, u32 h)
{
	unsigned int w_ratio;
	unsigned int h_ratio;
	int match;

	if (w == 0)
		return -1;
	w_ratio = (res->width << 13) / w;
	if (h == 0)
		return -1;
	h_ratio = (res->height << 13) / h;
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int fps_diff;
	int min_fps_diff = INT_MAX;
	int min_dist = INT_MAX;
	const struct tsb_resolution *tmp_res = NULL;
	struct tsb_device *dev = to_tsb_sensor(sd);

	for (i = 0; i < dev->entries_curr_table; i++) {
		tmp_res = &dev->curr_res_table[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
		if (dist == min_dist) {
			fps_diff = __tsb_min_fps_diff(dev->fps,
						tmp_res->fps_options);
			if (fps_diff < min_fps_diff) {
				min_fps_diff = fps_diff;
				idx = i;
			}
		}
	}

	return idx;
}

static int tsb_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int idx = 0;

	mutex_lock(&dev->input_lock);

	pr_info("%s, try %dx%d\n", __func__, fmt->width, fmt->height);

	if ((fmt->width > tsb_max_res[dev->sensor_id].res_max_width)
		|| (fmt->height > tsb_max_res[dev->sensor_id].res_max_height)) {
		fmt->width =  tsb_max_res[dev->sensor_id].res_max_width;
		fmt->height = tsb_max_res[dev->sensor_id].res_max_height;
	} else {
		idx = nearest_resolution_index(sd, fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 *  resolutions. If it fails, it means the requested
		 *  resolution is higher than wecan support. Fallback
		 *  to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = dev->entries_curr_table - 1;

		fmt->width = dev->curr_res_table[idx].width;
		fmt->height = dev->curr_res_table[idx].height;
	}

	pr_info("%s: select idx=%d, %dx%d\n", __func__, idx, fmt->width, fmt->height);

	fmt->code = dev->format.code;

	mutex_unlock(&dev->input_lock);
	return 0;
}

/* Call with ctrl_handler.lock hold */
static int __adjust_hvblank(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);
	u16 new_frame_length_lines, new_line_length_pck;
	int ret;

	/*
	 * No need to adjust h/v blank if not set dbg value
	 * Note that there is no other checking on the h/v blank value,
	 * as h/v blank can be set to any value above zero for debug purpose
	 */
	if (!dev->v_blank->val || !dev->h_blank->val)
		return 0;

	new_frame_length_lines = dev->curr_res_table[dev->fmt_idx].height +
		dev->v_blank->val;
	new_line_length_pck = dev->curr_res_table[dev->fmt_idx].width +
		dev->h_blank->val;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_LINE_LENGTH_PIXELS,
			    new_line_length_pck);
	if (ret)
		return ret;
	ret = tsb_write_reg(client, TSB_16BIT, TSB_FRAME_LENGTH_LINES,
			    new_frame_length_lines);
	if (ret)
		return ret;

	dev->lines_per_frame = new_frame_length_lines;
	dev->pixels_per_line = new_line_length_pck;

	return 0;
}

static int tsb_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct tsb_resolution *res;
	int ret;
	u16 val;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info == NULL)
		return -EINVAL;
	ret = tsb_try_mbus_fmt(sd, fmt);
	if (ret)
		return ret;

	mutex_lock(&dev->input_lock);

	pr_info("%s, width=%d, height=%d\n", __func__, fmt->width, fmt->height);
	dev->fmt_idx = nearest_resolution_index(sd, fmt->width, fmt->height);
	if (dev->fmt_idx == -1) {
		ret = -EINVAL;
		goto out;
	}
	res = &dev->curr_res_table[dev->fmt_idx];

	/* Adjust the FPS selection based on the resolution selected */
	dev->fps_index = __tsb_nearest_fps_index(dev->fps, res->fps_options);
	dev->fps = res->fps_options[dev->fps_index].fps;
	dev->regs = res->fps_options[dev->fps_index].regs;
	if (!dev->regs)
		dev->regs = res->regs;

	pr_info("%s, desc=%s\n", __func__, res->desc);

	tsb_write_reg_array(client, dev->mode_tables->init_settings);

	ret = tsb_write_reg_array(client, dev->regs);
	if (ret) {
		v4l2_err(client, "%s, write format array error\n", __func__);
		goto out;
	}

#ifdef TSB_BINNING_MODE
	if (BINNING_SUM == 1)
		tsb_write_reg(client, TSB_8BIT, TSB_BINNING_TYPE, 0x1);
	else
		tsb_write_reg(client, TSB_8BIT, TSB_BINNING_TYPE, 0x0);
#endif

	dev->pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	dev->lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;

	/* dbg h/v blank time */
	mutex_lock(dev->ctrl_handler.lock);
	__adjust_hvblank(sd);
	mutex_unlock(dev->ctrl_handler.lock);

	ret = __tsb_update_exposure_timing(client, dev->coarse_itg,
		dev->pixels_per_line, dev->lines_per_frame);
	if (ret) {
		v4l2_err(client, "%s, set exposure and gain error\n", __func__);
		goto out;
	}

	ret = tsb_write_reg_array(client, tsb_param_update);
	if (ret) {
		v4l2_err(client, "%s, param update error\n", __func__);
		goto out;
	}

	ret = tsb_get_intg_factor(client, tsb_info, dev->regs);
	if (ret) {
		v4l2_err(client, "%s, tsb_get_intg_factor error\n", __func__);
		goto out;
	}

	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	if (ret) {
		v4l2_err(client, "%s, read orientation error\n", __func__);
		goto out;
	}
	val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
	tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
	dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
out:
	mutex_unlock(&dev->input_lock);
	return ret;
}


static int tsb_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int tsb_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (tsb_read_reg(client, TSB_16BIT, TSB_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id != T4K37_ID && *id != T4K35_ID) {
		v4l2_err(client, "no toshiba sensor found\n");
		return -ENODEV;
	}
	v4l2_info(client, "sensor_id = 0x%x\n", *id);

	if (*id == T4K37_ID) {
		*id = T4K37_NAME_ID;
		ATD_t4k37_status = 1;
	} else
		ATD_t4k37_status = 0;

	/* TODO - need to be updated */
	*revision = 0;

	return 0;
}

static void __tsb_print_timing(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 width = dev->curr_res_table[dev->fmt_idx].width;
	u16 height = dev->curr_res_table[dev->fmt_idx].height;

	dev_dbg(&client->dev, "Dump tsb timing in stream on:\n");
	dev_dbg(&client->dev, "width: %d:\n", width);
	dev_dbg(&client->dev, "height: %d:\n", height);
	dev_dbg(&client->dev, "pixels_per_line: %d:\n", dev->pixels_per_line);
	dev_dbg(&client->dev, "line per frame: %d:\n", dev->lines_per_frame);
	dev_dbg(&client->dev, "pix freq: %d:\n", dev->vt_pix_clk_freq_mhz);
	dev_dbg(&client->dev, "init fps: %d:\n", dev->vt_pix_clk_freq_mhz /
			dev->pixels_per_line / dev->lines_per_frame);
	dev_dbg(&client->dev, "HBlank: %d nS:\n",
			1000 * (dev->pixels_per_line - width) /
			(dev->vt_pix_clk_freq_mhz / 1000000));
	dev_dbg(&client->dev, "VBlank: %d uS:\n",
			(dev->lines_per_frame - height) * dev->pixels_per_line /
			(dev->vt_pix_clk_freq_mhz / 1000000));
}

/*
 * tsb stream on/off
 */
static int tsb_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);

	pr_info("%s, enable=%d\n", __func__, enable);

	mutex_lock(&dev->input_lock);
	if (enable) {
		__tsb_print_timing(sd);
		ret = tsb_write_reg_array(client, tsb_streaming);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		dev->streaming = 1;
	} else {
		ret = tsb_write_reg_array(client, tsb_soft_standby);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		dev->streaming = 0;
		dev->fps_index = 0;
		dev->fps = 0;
	}
	mutex_unlock(&dev->input_lock);

	return 0;
}

/*
 * tsb enum frame size, frame intervals
 */
static int tsb_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct tsb_device *dev = to_tsb_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[index].width;
	fsize->discrete.height = dev->curr_res_table[index].height;
	fsize->reserved[0] = dev->curr_res_table[index].used;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int tsb_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;
	struct tsb_device *dev = to_tsb_sensor(sd);

	mutex_lock(&dev->input_lock);
	/* since the isp will donwscale the resolution to the right size,
	  * find the nearest one that will allow the isp to do so
	  * important to ensure that the resolution requested is padded
	  * correctly by the requester, which is the atomisp driver in
	  * this case.
	  */
	i = nearest_resolution_index(sd, fival->width, fival->height);

	if (i == -1)
		goto out;

	/* Check if this index is supported */
	if (index > __tsb_get_max_fps_index(dev->curr_res_table[i].fps_options))
		goto out;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = dev->curr_res_table[i].width;
	fival->height = dev->curr_res_table[i].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->curr_res_table[i].fps_options[index].fps;
	mutex_unlock(&dev->input_lock);
	return 0;
out:
	mutex_unlock(&dev->input_lock);
	return -EINVAL;
}

static int tsb_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int __update_tsb_device_settings(struct tsb_device *dev, u16 sensor_id)
{
	pr_info("%s, sensor_id=0x%x\n", __func__, sensor_id);

	switch (sensor_id) {
	case T4K37_NAME_ID:
		dev->mode_tables = &tsb_sets[T4K37_MERRFLD];
		dev->vcm_driver = &tsb_vcms[T4K37_MERRFLD];
		break;
	case T4K35_NAME_ID:
		dev->mode_tables = &tsb_sets[T4K35_MERRFLD];
		dev->vcm_driver = &tsb_vcms[T4K35_MERRFLD];
		break;
	default:
		dev->mode_tables = &tsb_sets[T4K37_MERRFLD];
		dev->vcm_driver = &tsb_vcms[T4K37_MERRFLD];
		pr_info("%s: sensorid=%d, but use default t4k37's setting\n",
			__func__, sensor_id);
		break;
	}

	return dev->vcm_driver->init(&dev->sd);
}

static int tsb_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	pr_info("%s+\n", __func__);

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	ret = __tsb_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "tsb power-up err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = tsb_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "tsb_detect err s_config.\n");
#if DEBUG_TEMP
		goto fail_detect;
#endif
	}

	dev->sensor_id = sensor_id;
	pr_info("%s, sensor_id=0x%x\n", __func__, dev->sensor_id);
	dev->sensor_revision = sensor_revision;

	/* Resolution settings depend on sensor type and platform */
	ret = __update_tsb_device_settings(dev, dev->sensor_id);
	if (ret)
		goto fail_detect;

#if DEBUG_TEMP
	/* Read sensor's OTP data */
	dev->otp_driver->otp_read(sd, tsb_otp_data);
	memcpy(&tsb_otp, &tsb_otp_data, sizeof(tsb_otp_data));
#endif

	/* power off sensor */
	ret = __tsb_s_power(sd, 0);

	mutex_unlock(&dev->input_lock);
	if (ret)
		v4l2_err(client, "tsb power-down err.\n");

	pr_info("%s-\n", __func__);
	return ret;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__tsb_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
tsb_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
tsb_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	struct tsb_device *dev = to_tsb_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fse->min_width = dev->curr_res_table[index].width;
	fse->min_height = dev->curr_res_table[index].height;
	fse->max_width = dev->curr_res_table[index].width;
	fse->max_height = dev->curr_res_table[index].height;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static struct v4l2_mbus_framefmt *
__tsb_get_pad_format(struct tsb_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
tsb_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__tsb_get_pad_format(dev, fh, fmt->pad, fmt->which);

	fmt->format = *format;

	return 0;
}

static int
tsb_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
tsb_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		dev->curr_res_table = dev->mode_tables->res_video;
		dev->entries_curr_table = dev->mode_tables->n_res_video;
		break;
	case CI_MODE_STILL_CAPTURE:
		dev->curr_res_table = dev->mode_tables->res_still;
		dev->entries_curr_table = dev->mode_tables->n_res_still;
		break;
	default:
		dev->curr_res_table = dev->mode_tables->res_preview;
		dev->entries_curr_table = dev->mode_tables->n_res_preview;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

int
tsb_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	const struct tsb_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];

	mutex_lock(&dev->input_lock);
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int __tsb_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct tsb_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];
	struct camera_mipi_info *tsb_info = NULL;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	unsigned int fps_index;
	int fps;
	int ret = 0;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info == NULL)
		return -EINVAL;

	if (!interval->interval.numerator)
		interval->interval.numerator = 1;

	fps = interval->interval.denominator / interval->interval.numerator;

	if (!fps)
		/* currently does not support fps format like 1/2, 1/3 */
		fps = 1;

	/* No need to proceed further if we are not streaming */
	if (!dev->streaming) {
		/* Save the new FPS and use it while selecting setting */
		dev->fps = fps;
		return 0;
	}

	 /* Ignore if we are already using the required FPS. */
	if (fps == dev->fps)
		return 0;

	/*
	 * Start here, sensor is already streaming, so adjust fps dynamically
	 */
	fps_index = __tsb_above_nearest_fps_index(fps, res->fps_options);

	if (res->fps_options[fps_index].regs &&
	    res->fps_options[fps_index].regs != dev->regs) {
		/*
		 * if need a new setting, but the new setting has difference
		 * with current setting, not use this one, as may have
		 * unexpected result, e.g. PLL, IQ.
		 */
		dev_dbg(&client->dev, "Sensor is streaming, not apply new sensor setting\n");
		if (fps > res->fps_options[dev->fps_index].fps) {
			/*
			 * Does not support increase fps based on low fps
			 * setting, as the high fps setting could not be used,
			 * and fps requested is above current setting fps.
			 */
			dev_warn(&client->dev, "Could not support fps: %d, keep current: %d.\n",
					fps, dev->fps);
			goto done;
		}
	} else {
		dev->fps_index = fps_index;
		dev->fps = res->fps_options[dev->fps_index].fps;
	}

	/* Update the new frametimings based on FPS */
	pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;

	if (fps > res->fps_options[fps_index].fps) {
		/*
		 * if does not have high fps setting, not support increase fps
		 * by adjust lines per frame.
		 */
		dev_warn(&client->dev, "Could not support fps: %d. Use:%d.\n",
				fps, res->fps_options[fps_index].fps);
		goto update;
	}

	/* if the new setting does not match exactly */
	if (dev->fps != fps) {
#define MAX_LINES_PER_FRAME	0xffff
		dev_dbg(&client->dev, "adjusting fps using lines_per_frame\n");
		/*
		 * FIXME!
		 * 1: check DS on max value of lines_per_frame
		 * 2: consider use pixel per line for more range?
		 */
		if (dev->lines_per_frame * dev->fps / fps >
				MAX_LINES_PER_FRAME) {
			dev_warn(&client->dev,
					"adjust lines_per_frame out of range, try to use max value.\n");
			lines_per_frame = MAX_LINES_PER_FRAME;
		} else {
			lines_per_frame = lines_per_frame * dev->fps / fps;
		}
	}
update:
	/* Update the new frametimings based on FPS */
	dev->pixels_per_line = pixels_per_line;
	dev->lines_per_frame = lines_per_frame;

	/* Update the new values so that user side knows the current settings */
	ret = __tsb_update_exposure_timing(client,
		dev->coarse_itg, dev->pixels_per_line, dev->lines_per_frame);
	if (ret)
		return ret;

	dev->fps = fps;

	ret = tsb_get_intg_factor(client, tsb_info, dev->regs);
	if (ret)
		return ret;
done:
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	__tsb_print_timing(sd);

	return ret;
}

static int tsb_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __tsb_s_frame_interval(sd, interval);
	mutex_unlock(&dev->input_lock);

	return ret;
}
static int tsb_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = dev->curr_res_table[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops tsb_sensor_ops = {
	.g_skip_frames	= tsb_g_skip_frames,
};

static int tsb_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int tsb_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tsb_device *dev = container_of(ctrl->handler, struct tsb_device,
			ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ctrl->val = dev->lines_per_frame -
			dev->curr_res_table[dev->fmt_idx].height;
		break;
	case V4L2_CID_HBLANK:
		ctrl->val = dev->pixels_per_line -
			dev->curr_res_table[dev->fmt_idx].width;
		break;
	case V4L2_CID_PIXEL_RATE:
		ctrl->val = dev->vt_pix_clk_freq_mhz;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct v4l2_ctrl_ops tsb_ctrl_ops = {
	.s_ctrl = tsb_set_ctrl,
	.g_volatile_ctrl = tsb_g_volatile_ctrl,
};

static const struct v4l2_subdev_video_ops tsb_video_ops = {
	.s_stream = tsb_s_stream,
	.enum_framesizes = tsb_enum_framesizes,
	.enum_frameintervals = tsb_enum_frameintervals,
	.enum_mbus_fmt = tsb_enum_mbus_fmt,
	.try_mbus_fmt = tsb_try_mbus_fmt,
	.g_mbus_fmt = tsb_g_mbus_fmt,
	.s_mbus_fmt = tsb_s_mbus_fmt,
	.s_parm = tsb_s_parm,
	.g_frame_interval = tsb_g_frame_interval,
	.s_frame_interval = tsb_s_frame_interval,
};

static const struct v4l2_subdev_core_ops tsb_core_ops = {
	.queryctrl = tsb_queryctrl,
	.g_ctrl = tsb_g_ctrl,
	.s_ctrl = tsb_s_ctrl,
	.s_power = tsb_s_power,
	.ioctl = tsb_ioctl,
	.init = tsb_init,
};

static const struct v4l2_subdev_pad_ops tsb_pad_ops = {
	.enum_mbus_code = tsb_enum_mbus_code,
	.enum_frame_size = tsb_enum_frame_size,
	.get_fmt = tsb_get_pad_format,
	.set_fmt = tsb_set_pad_format,
};

static const struct v4l2_subdev_ops tsb_ops = {
	.core = &tsb_core_ops,
	.video = &tsb_video_ops,
	.pad = &tsb_pad_ops,
	.sensor = &tsb_sensor_ops,
};

static const struct media_entity_operations tsb_entity_ops = {
	.link_setup = NULL,
};

static int tsb_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tsb_device *dev = to_tsb_sensor(sd);

	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

/*++++++++++ dbgfs ++++++++++*/

static int dbg_dump_tsb_otp_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_tsb_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,
	"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\
	\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\
	\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\
	\n0x%02X 0x%02X\n",
	tsb_otp_data[0], tsb_otp_data[1], tsb_otp_data[2], tsb_otp_data[3], tsb_otp_data[4],
	tsb_otp_data[5], tsb_otp_data[6], tsb_otp_data[7], tsb_otp_data[8], tsb_otp_data[9],
	tsb_otp_data[10], tsb_otp_data[11], tsb_otp_data[12], tsb_otp_data[13], tsb_otp_data[14],
	tsb_otp_data[15], tsb_otp_data[16], tsb_otp_data[17], tsb_otp_data[18], tsb_otp_data[19],
	tsb_otp_data[20], tsb_otp_data[21], tsb_otp_data[22], tsb_otp_data[23], tsb_otp_data[24],
	tsb_otp_data[25], tsb_otp_data[26], tsb_otp_data[27], tsb_otp_data[28], tsb_otp_data[29],
	tsb_otp_data[30], tsb_otp_data[31]);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_tsb_otp_fops = {
	.open		= dbg_dump_tsb_otp_open,
	.read		= dbg_dump_tsb_otp_read,
};

static int dbg_dump_tsb_res_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_tsb_res_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", T4K37_RESOLUTION);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_tsb_res_fops = {
	.open		= dbg_dump_tsb_res_open,
	.read		= dbg_dump_tsb_res_read,
};

static int dbg_dump_tsb_module_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_tsb_module_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", T4K37_MODULE);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_tsb_module_fops = {
	.open		= dbg_dump_tsb_module_open,
	.read		= dbg_dump_tsb_module_read,
};

static int dbg_dump_tsb_uid_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_tsb_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,
		"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		tsb_otp.dc[0], tsb_otp.dc[1],  tsb_otp.dc[2],  tsb_otp.dc[3],
		tsb_otp.sn[0], tsb_otp.sn[1],  tsb_otp.sn[2],  tsb_otp.sn[3],
		tsb_otp.pn[0], tsb_otp.pn[1],  tsb_otp.pn[2],  tsb_otp.pn[3]);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_tsb_uid_fops = {
	.open		= dbg_dump_tsb_uid_open,
	.read		= dbg_dump_tsb_uid_read,
};

static int dbg_set_tsb_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_tsb_reg_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int ofst = 0, reg_val = 0;
	u16 reg, val;
	int buf_count = 0;

	u8 i2c_buf[16];
	u8 *b_ptr = i2c_buf;
	int err = 0;

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &ofst, &reg_val);

	reg = ofst & 0xFFFF;
	val = reg_val & 0xFF;

	*b_ptr++ = reg_val;
	buf_count++;
	pr_info("write [0x%x]= 0x%x\n", reg, val);
	err = tsb_write_reg(g_client, TSB_8BIT, reg, val);
	if (err)
		pr_info("dbg write error\n");


	return count;
}

static ssize_t dbg_set_tsb_reg_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 value = 0xFF;
	int err = 0;

	if (*ppos)
		return 0;	/* the end */

	/* read register */
	read_register = read_register & 0xFFFF;
	err = tsb_read_reg(g_client, TSB_8BIT,
					read_register, &value);
	pr_info("%s, read [0x%x] = 0x%x\n", __func__, read_register, value);

	if (err) {
		len = snprintf(bp, dlen, "error\n");
		tot += len; bp += len; dlen -= len;
		pr_info("%s, err=%d\n", __func__, err);
	} else {
		len = snprintf(bp, dlen, "0x%x\n", value);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_set_tsb_reg_fops = {
	.open		= dbg_set_tsb_reg_open,
	.write		= dbg_set_tsb_reg_write,
	.read		= dbg_set_tsb_reg_read,
};

static int dbg_set_tsb_read_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_tsb_read_reg_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int ofst = 0;
	u16 reg;

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x", &ofst);

	reg = ofst & 0xFFFF;
	read_register = reg;

	return count;
}

static ssize_t dbg_set_tsb_read_reg_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "0x%x\n", read_register);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_set_tsb_read_reg_fops = {
	.open		= dbg_set_tsb_read_reg_open,
	.write		= dbg_set_tsb_read_reg_write,
	.read		= dbg_set_tsb_read_reg_read,
};

static int dbg_dump_tsb_read_sensor_id_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_tsb_read_sensor_id_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	struct v4l2_subdev *sd = i2c_get_clientdata(g_client);
	u8 sensor_revision;
	u16 sensor_id;

	/* pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos); */

	if (*ppos)
		return 0;	/* the end */

	ret = power_up(sd);

	ret |= tsb_detect(g_client, &sensor_id, &sensor_revision);
	if (ret)
		pr_err("tsb read sensor id fail\n");

	ret |= power_down(sd);

	len = snprintf(bp, dlen, "%d\n", ret);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_tsb_read_sensor_id_fops = {
	.open		= dbg_dump_tsb_read_sensor_id_open,
	.read		= dbg_dump_tsb_read_sensor_id_read,
};

static int tsb_dbgfs_init(void)
{
	struct dentry *debugfs_dir;

	/* ATD command */
	debugfs_dir = debugfs_create_dir("camera0", NULL);
	debugfs_create_u32("camera_status", 0644, debugfs_dir, &ATD_t4k37_status);
	debugfs_create_u32("vcm", 0644, debugfs_dir, &g_vcm_pos);
	(void) debugfs_create_file("CameraOTP", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_tsb_otp_fops);
	(void) debugfs_create_file("CameraResolution", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_tsb_res_fops);
	(void) debugfs_create_file("CameraModule", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_tsb_module_fops);
	(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_tsb_uid_fops);

	/* do not set exposure */
	debugfs_create_u32("exposure_return0", 0666, debugfs_dir, &exposure_return0);

	/* i2c register read/write */
	(void) debugfs_create_file("read_register", S_IRUGO | S_IWUSR,
		debugfs_dir, NULL, &dbg_set_tsb_read_reg_fops);
	(void) debugfs_create_file("i2c_write_read", S_IRUGO | S_IWUSR,
		debugfs_dir, NULL, &dbg_set_tsb_reg_fops);

	/* i2c test - power on, read sensor id, power off */
	(void) debugfs_create_file("SensorID", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_tsb_read_sensor_id_fops);

	return 0;
}
/*---------- dbgfs ----------*/

static int __tsb_init_ctrl_handler(struct tsb_device *dev)
{
	struct v4l2_ctrl_handler *hdl;

	hdl = &dev->ctrl_handler;

	v4l2_ctrl_handler_init(&dev->ctrl_handler, 3);

	dev->pixel_rate = v4l2_ctrl_new_std(&dev->ctrl_handler,
					    &tsb_ctrl_ops,
					    V4L2_CID_PIXEL_RATE,
					    0, UINT_MAX, 1, 0);

	dev->h_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &tsb_ctrl_ops,
					  V4L2_CID_HBLANK, 0, SHRT_MAX, 1, 0);

	dev->v_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &tsb_ctrl_ops,
					  V4L2_CID_VBLANK, 0, SHRT_MAX, 1, 0);

	if (dev->ctrl_handler.error || dev->pixel_rate == NULL
		|| dev->h_blank == NULL || dev->v_blank == NULL) {
		return dev->ctrl_handler.error;
	}

	dev->sd.ctrl_handler = hdl;

	dev->pixel_rate->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->h_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->v_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;

	return 0;
}

static ssize_t t4k37_ctl_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", ATD_t4k37_status);
}
static DEVICE_ATTR(t4k37_ctl_status, S_IRUGO, t4k37_ctl_status_show, NULL);

static struct kobject *sys_kobj;
static ssize_t cam_check_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct v4l2_subdev *sd = i2c_get_clientdata(g_client);
	u8 sensor_revision;
	u16 sensor_id;
	mutex_lock(&g_dev->input_lock);
	ret = power_up(sd);

	ret |= tsb_detect(g_client, &sensor_id, &sensor_revision);
	if (ret)
		pr_err("tsb read sensor id fail\n");

	ret |= power_down(sd);
	mutex_unlock(&g_dev->input_lock);

	return snprintf(buf, sizeof("%d\n"), "%d\n", ret);
}

static struct kobj_attribute sys_file_attribute =
	__ATTR(check, 0444, cam_check_show, NULL);

static struct attribute *sys_cam_check_attr[] = {
	&sys_file_attribute.attr,
	NULL,
};

static struct attribute_group sys_file_cam_group = {
     .name = "cam_check",
     .attrs = sys_cam_check_attr,
};

static int tsb_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tsb_device *dev;
	struct camera_mipi_info *tsb_info = NULL;
	int i2c;
	int ret = 0;
	void *pdata = client->dev.platform_data;

	pr_info("%s+\n", __func__);

	/* Firmware workaround: Some modules use a "secondary default"
	 * address of 0x10 which doesn't appear on schematics, and
	 * some BIOS versions haven't gotten the memo.  Work around
	 * via config. */
	i2c = gmin_get_var_int(&client->dev, "I2CAddr", -1);
	if (i2c != -1) {
		dev_info(&client->dev,
		"Overriding firmware-provided I2C address (0x%x) with 0x%x\n",
			 client->addr, i2c);
		client->addr = i2c;
	}

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	dev->sensor_id = TSB_ID_DEFAULT;
	dev->vcm_driver = &tsb_vcms[TSB_ID_DEFAULT];
	dev->otp_driver = &tsb_otps;

	v4l2_i2c_subdev_init(&(dev->sd), client, &tsb_ops);

	if (ACPI_COMPANION(&client->dev))
		pdata = gmin_camera_platform_data(&dev->sd,
						  ATOMISP_INPUT_FORMAT_RAW_10,
						  atomisp_bayer_order_grbg);
	if (!pdata)
		goto out_free;

	ret = tsb_s_config(&dev->sd, client->irq, pdata);
#if DEBUG_TEMP
	if (ret)
		goto out_free;
#endif

	ret = atomisp_register_i2c_module(&dev->sd, pdata, RAW_CAMERA);
	if (ret)
		goto out_free;

	tsb_info = v4l2_get_subdev_hostdata(&dev->sd);

	/*
	 * sd->name is updated with sensor driver name by the v4l2.
	 * change it to sensor name in this case.
	 */
	snprintf(dev->sd.name, sizeof(dev->sd.name), "%s%x %d-%04x",
		TSB_SUBDEV_PREFIX, dev->sensor_id,
		i2c_adapter_id(client->adapter), client->addr);

	ret = __tsb_init_ctrl_handler(dev);
	if (ret)
		goto out_ctrl_handler_free;

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
	dev->sd.entity.ops = &tsb_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		tsb_remove(client);
		return ret;
	}

	/* dbgfs for ATD */
	g_client = client;
	tsb_dbgfs_init();

	// sysfs for monitor
	g_dev = dev;
	sys_kobj = kobject_create_and_add("camera_monitor", NULL);
	if(sys_kobj) {
		ret = sysfs_create_group(sys_kobj, &sys_file_cam_group);
		if (ret) {
			pr_err("%s Failed to crate sys_file_cam_group sysfs files %d\n", __func__, ret);
		}
	} else {
		pr_err("%s Failed to crate sys_kobj\n", __func__);
	}

	ret = device_create_file(&client->dev, &dev_attr_t4k37_ctl_status);
	if (ret != 0) {
		dev_err(&client->dev, "Failed to crate t4k37_ctl_status sysfs files %d\n", ret);
		return ret;
	}

	pr_info("%s-\n", __func__);
	return ret;

out_ctrl_handler_free:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	pr_info("%s error out\n", __func__);
	return ret;
}

static const struct i2c_device_id tsb_ids[] = {
	{TSB_NAME_K37, T4K37_NAME_ID},
	{TSB_NAME_K35, T4K35_NAME_ID},
	{}
};
MODULE_DEVICE_TABLE(i2c, tsb_ids);

static struct acpi_device_id t4k37_acpi_match[] = {
	{"INT33BA"},
	{},
};
MODULE_DEVICE_TABLE(acpi, t4k37_acpi_match);

static struct i2c_driver tsb_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TSB_DRIVER,
		.acpi_match_table = ACPI_PTR(t4k37_acpi_match),
	},
	.probe = tsb_probe,
	.remove = tsb_remove,
	.id_table = tsb_ids,
};


static __init int init_tsb(void)
{
	return i2c_add_driver(&tsb_driver);
}

static __exit void exit_tsb(void)
{
	if(sys_kobj) {
		sysfs_remove_group(sys_kobj, &sys_file_cam_group);
		kobject_put(sys_kobj);
	}

	i2c_del_driver(&tsb_driver);
}

module_init(init_tsb);
module_exit(exit_tsb);

MODULE_DESCRIPTION("A low-level driver for Toshiba sensors");
MODULE_AUTHOR("Shenbo Huang <shenbo.huang@intel.com>");
MODULE_LICENSE("GPL");

