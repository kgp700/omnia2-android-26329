/* drivers/input/touchscreen/melfas_ts_i2c_tsi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>

#include <asm/io.h>
#include <linux/irq.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>

//#include <linux/dprintk.h>

//#define MELFAS_I2C_ADDR 0x40
#define IRQ_TOUCH_INT IRQ_EINT(8)

#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_VITAL) || defined(CONFIG_MACH_MAX)
// Current VinsQ's TSP IC is not fully capable of generating dual touch points
// So instead, it gives one point and a distance from that point.
// Below feature enables "LIMITED" Multi Touch functionality.
// What it does is when TSP gives a point and a distance, it will generate
// a second point using the distance.
// For example if the received touch information are a touch point of (100,100)
// and a distance of 50, a second touch point of (100,150) will be reported.
// This just works fine for the pinch-zoom in/out gestures because all it needs
// is a increase or decrease in distance of the two points.
//
// NOTE that it will not work for any other multi-touch gestures!!
//
#define USES_PINCH_DIST_MT
#endif

#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

static int debug_level;
#define debugprintk(level,x...)  if(debug_level>=level) printk(x)
//#define debugprintk(level,x...)  printk(x)

#if defined(CONFIG_MACH_VINSQ)
static int enhance_oj=0;
#endif
extern int mcsdl_download_binary_data(int hw_ver);
extern int get_sending_oj_event();

struct input_info {
	int max_x;
	int max_y;
    int state;
    int x;
    int y;
    int z;
    int width;
};

struct melfas_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	int mt_support; // multitouch support
	int mt_disable;
    struct input_info info;
    int suspended;
	struct early_suspend	early_suspend;
};

struct melfas_ts_driver *melfas_ts = NULL;
struct i2c_driver melfas_ts_i2c;
struct workqueue_struct *melfas_ts_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h);
void melfas_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#define TOUCH_HOME	KEY_HOME
#define TOUCH_MENU	KEY_MENU
#define TOUCH_BACK	KEY_BACK
#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_MAX) || defined(CONFIG_MACH_VITAL)
#define TOUCH_SEARCH  KEY_SEARCH
#endif

int melfas_ts_tk_keycode[] =
#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_MAX) || defined(CONFIG_MACH_VITAL)
{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, TOUCH_SEARCH, };
#else
{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, };
#endif

extern struct class *sec_class;
struct device *ts_dev;

int melfas_check_mt_support(int do_update)
{
	static int first = true;
#if defined(CONFIG_MACH_VINSQ) && defined(USES_PINCH_DIST_MT)
	if(do_update || first) {
		if((((melfas_ts->hw_rev==0x40)||(melfas_ts->hw_rev==0x50))&&(melfas_ts->fw_ver >= 0x16))
			|| (((melfas_ts->hw_rev==0x80)||(melfas_ts->hw_rev==0xA0))&&(melfas_ts->fw_ver >= 0x15))
			|| (melfas_ts->hw_rev>0xA0))
			melfas_ts->mt_support = true;
		else
			melfas_ts->mt_support = false;

		first = false;
	}
	return melfas_ts->mt_support;
#elif defined(CONFIG_MACH_VITAL) && defined(USES_PINCH_DIST_MT)
	if(do_update || first) {
#if 0//kimhyuns from Vital TSP hw, fw support multi touch
		if((melfas_ts->hw_rev==0x20)&&(melfas_ts->fw_ver >= 0x2))
			melfas_ts->mt_support = true;
		else
			melfas_ts->mt_support = false;
#else
		melfas_ts->mt_support = true;
#endif
		first = false;
	}
	return melfas_ts->mt_support;
#elif defined(CONFIG_MACH_MAX) && defined(USES_PINCH_DIST_MT)
	if(do_update || first) {
		if(melfas_ts->hw_rev == 0x61)
			melfas_ts->mt_support = true;
		else
			melfas_ts->mt_support = false;

		first = false;
	}
	return melfas_ts->mt_support;
#else
	if(do_update || first) {
		melfas_ts->mt_support = 0;
	}
	return melfas_ts->mt_support;
#endif
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status, op_mode, hw_rev, fw_ver;

	status  = i2c_smbus_read_byte_data(melfas_ts->client, 0x00);
	if (status < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	op_mode = i2c_smbus_read_byte_data(melfas_ts->client, 0x01);
	if (op_mode < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, 0x1E);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, 0x20);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}

	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status) : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (op_mode): 0x%08x\n", buf, op_mode);
	sprintf(buf, "%sRegister 0x1E (hw_rev) : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x20 (fw_ver) : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {

	    ret = i2c_smbus_write_byte_data(melfas_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_EN  : %s\n", buf, gpio_get_value(GPIO_TOUCH_EN)? "HIGH":"LOW");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW");
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
		gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
		gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
		printk("[TOUCH] disable.\n");
	}

	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
		mdelay(500);
		gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;;

	int hw_rev, fw_ver;

	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, 0x1E);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, 0x20);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}

	sprintf(buf, "H/W rev. 0x%x F/W ver. 0x%x\n", hw_rev, fw_ver);
	return sprintf(buf, "%s", buf);
}

static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE
	int ret;
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0
		|| strncmp(buf, "forceupdate", 11) == 0) {
		printk("[TOUCH] HW version: 0x%x.\n", melfas_ts->hw_rev);
		printk("[TOUCH] Current FW version: 0x%x.\n", melfas_ts->fw_ver);
		s3c_gpio_cfgpin(GPIO_I2C0_SDA, S3C_GPIO_SFN(1));
		s3c_gpio_cfgpin(GPIO_I2C0_SCL, S3C_GPIO_SFN(1));

		if(strncmp(buf, "forceupdate", 11) == 0) {
			ret = mcsdl_download_binary_data(-1);
		}
		else {
		ret = mcsdl_download_binary_data(melfas_ts->hw_rev);
		}
		s3c_gpio_cfgpin(GPIO_I2C0_SDA, S3C_GPIO_SFN(GPIO_I2C0_SDA_AF));
		s3c_gpio_cfgpin(GPIO_I2C0_SCL, S3C_GPIO_SFN(GPIO_I2C0_SCL_AF));
		if(ret) {
			int hw_rev, fw_ver;
			hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, 0x1E);
			if (hw_rev < 0) {
				printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
			}
			else {
				melfas_ts->hw_rev = hw_rev;
			}
			fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, 0x20);
			if (fw_ver < 0) {
				printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
			}
			else {
				melfas_ts->fw_ver = fw_ver;
			}
			printk("[TOUCH] firmware update success!(hw=%x, fw=%x)\n", hw_rev, fw_ver);
		}
		else {
			printk("[TOUCH] firmware update failed.. RESET!\n");
			gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
			mdelay(500);
			gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
			mdelay(200);
		}
	}
#endif

	return size;
}


static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", debug_level);
}
static ssize_t debug_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    if(buf[0]>'0' && buf[0]<='9') {
        debug_level = buf[0] - '0';
    }

    return size;
}

#if defined(CONFIG_MACH_VINSQ)
static ssize_t enhance_oj_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s", enhance_oj?"1":"0");
	
}
static ssize_t enhance_oj_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "1", 1) == 0) {
		enhance_oj = 1;
		printk("[TSP] OJ enhancement enabled.\n");
	}
	else if(strncmp(buf, "0", 1) == 0) {
		enhance_oj = 0;
		printk("[TSP] OJ enhancement  disabled.\n");
	}
}
#endif

#ifdef USES_PINCH_DIST_MT
static ssize_t multitouch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s", melfas_ts->mt_disable?"disabled":"enabled");
}
static ssize_t multitouch_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "enable", 6) == 0) {
		melfas_ts->mt_disable = false;
		printk("[TSP] Multi-touch enabled\n");
	}
	else if(strncmp(buf, "disable", 7) == 0) {
		melfas_ts->mt_disable = true;
		printk("[TSP] Multi-touch disabled\n");
	}

    return size;
}
static DEVICE_ATTR(multitouch, S_IRUGO | S_IWUSR, multitouch_show, multitouch_store);
#endif

static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show, registers_store);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR, firmware_show, firmware_store);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);

#if defined(CONFIG_MACH_VINSQ)
static DEVICE_ATTR(enhance_oj, S_IRUGO | S_IWUSR, enhance_oj_show, enhance_oj_store);
#endif


#define INPUT_INFO_REG 0x10
void melfas_ts_work_func(struct work_struct *work)
{
  int ret;
  struct i2c_msg msg[2];

  struct i2c_msg msg1[2];
  uint8_t buf2[1];
  int button;
  unsigned int keycode;
  static unsigned int keypress = 0;;

  uint8_t start_reg;
  uint8_t buf1[8];
  int	x_old, y_old;
  x_old = y_old = 0;

  msg[0].addr = melfas_ts->client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = INPUT_INFO_REG;
  msg[1].addr = melfas_ts->client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;

  ret = i2c_transfer(melfas_ts->client->adapter, msg, 2);

  if (ret < 0)
  {
    printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
  }
  else
  {
    int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8;
    int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8;
    int z = buf1[5];
    int finger = buf1[0] & 0x01;
    int width = buf1[6];
    int touchtype = buf1[0] & (BIT(0)|BIT(1)|BIT(2));
#ifdef USES_PINCH_DIST_MT
    int gesture_code = (buf1[0] & (BIT(3)|BIT(4)|BIT(5)))>>3;
	static int x2,y2,z2;
#endif

#ifdef CONFIG_CPU_FREQ
    set_dvfs_perf_level();
#endif

#if defined(CONFIG_MACH_VINSQ)
    if(buf1[0] == 0x80) {
        // ESD !!!
        // turn TSP power off => (100ms) => turn TSP power back on

        printk(KERN_ERR "[TSP] ESD detected resetting TSP!!!");
        gpio_set_value(GPIO_TOUCH_EN, 0);
        mdelay(100);
        gpio_set_value(GPIO_TOUCH_EN, 1);

        enable_irq(melfas_ts->irq);
        return;
    }
#elif defined(CONFIG_MACH_VITAL)
    if(buf1[0] == 0x80) {
        // ESD !!!
        // turn TSP power off => (100ms) => turn TSP power back on

        printk(KERN_ERR "[TSP] ESD detected resetting TSP!!!");
        gpio_set_value(GPIO_TOUCH_EN, 0);
        mdelay(100);
        gpio_set_value(GPIO_TOUCH_EN, 1);

        enable_irq(melfas_ts->irq);
        return;
    }
#elif defined(CONFIG_MACH_MAX)
    if(buf1[0] == 0x05) {
        // ESD !!!

        printk(KERN_ERR "[TSP] ESD detected resetting TSP!!!");
        gpio_set_value(GPIO_TOUCH_EN, 0);
        mdelay(200);
        gpio_set_value(GPIO_TOUCH_EN, 1);

        enable_irq(melfas_ts->irq);
        return;
    }
#endif

#if !defined(CONFIG_MACH_MAX)
    if(buf1[0] & 0xC0) {
      msg1[0].addr = melfas_ts->client->addr;
      msg1[0].flags = 0;
      msg1[0].len = 1;
      msg1[0].buf = &start_reg;
      start_reg = 0x25;
      msg1[1].addr = melfas_ts->client->addr;
      msg1[1].flags = I2C_M_RD;
      msg1[1].len = sizeof(buf2);
      msg1[1].buf = buf2;

      ret = i2c_transfer(melfas_ts->client->adapter, msg1, 2);

      button = buf2[0]; //key:1 home key:2 menu key:3 back

      switch(button) {
        case 0x01 :
        case 0x09 :
#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_VITAL)
          keycode = TOUCH_MENU;
#else
          keycode = TOUCH_HOME;
#endif
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;

        case 0x02 :
        case 0x0A :
#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_VITAL)
          keycode = TOUCH_HOME;
#else
          keycode = TOUCH_MENU;
#endif
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;

        case 0x03 :
        case 0x0B :
          keycode = TOUCH_BACK;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;

#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_VITAL)
        case 0x04 :
        case 0x0C :
          keycode = TOUCH_SEARCH;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;
#endif

        default :
          printk("[TOUCH_KEY] undefined button: 0x%02x.\n", button);
          enable_irq(melfas_ts->irq);
          return;
      }
#if defined(CONFIG_MACH_VINSQ)	
	if(!((keycode == TOUCH_HOME || keycode == TOUCH_BACK) && (get_sending_oj_event()&&enhance_oj)))
#endif /* CONFIG_MACH_MAX */
	{
		if(button & 0x08)
			keypress = 0;
		else
			keypress = 1;
		printk("[TOUCH_KEY] keycode: %4d, keypress: %4d\n", keycode, keypress);
		input_report_key(melfas_ts->input_dev, keycode, keypress);
	}
#if defined(CONFIG_MACH_VINSQ)
	else {
		if(keypress) {
			if(button & 0x08) {
				keypress = 0;
				printk("[TOUCH_KEY] keycode: %4d, keypress: %4d\n", keycode, keypress);
				input_report_key(melfas_ts->input_dev, keycode, keypress);
			}
		}	
	}
#endif
    }
    else
#endif
    {
        int do_report = false;
#ifdef USES_PINCH_DIST_MT
		static int do_report2 = false; // report a second touch point
#endif

        switch(touchtype) {
            case 0x0: // Non-touched state
                melfas_ts->info.x = -1;
                melfas_ts->info.y = -1;
                melfas_ts->info.z = -1;
                melfas_ts->info.width = -1;
				z = 0;
                do_report = true;
				#ifdef USES_PINCH_DIST_MT
				if(do_report2 == true) {
					z2 = 0;
				}
				#endif
                break;
            case 0x1: // Single-point Touch
                melfas_ts->info.x = x;
                melfas_ts->info.y = y;
                melfas_ts->info.z = z;
                melfas_ts->info.width = width;
                do_report = true;
				#ifdef USES_PINCH_DIST_MT
				do_report2 = false;
				#endif
                break;
            case 0x2: // Dual-point Touch
				#ifdef USES_PINCH_DIST_MT
				if(likely(melfas_ts->mt_support==true) && melfas_ts->mt_disable==false) {
					if(gesture_code) {
						melfas_ts->info.x = x;
						melfas_ts->info.y = y;
						melfas_ts->info.z = z;
						x2 = x;
						width=width<<1;
						if(unlikely(width > melfas_ts->info.max_y))
							width = melfas_ts->info.max_y;
						if(y+width > melfas_ts->info.max_y)
							y2 = y - width;
						else
							y2 = y + width;
						z2 = z;
						do_report = true;
						do_report2 = true;
					}
				}
				#endif
                printk(KERN_DEBUG "[TSP] Dual-point Touch %s!\n",melfas_ts->mt_support?"received":"ignored");
                break;
            case 0x3: // Palm Touch
                printk(KERN_DEBUG "[TSP] Palm Touch!\n");
                break;
            case 0x7: // Proximity
                printk(KERN_DEBUG "[TSP] Proximity!\n");
                break;
        }

        melfas_ts->info.state = touchtype;

        if(do_report) {

			#ifdef USES_PINCH_DIST_MT
			if(likely(melfas_ts->mt_support == true)) {
				debugprintk(5,"[ABS_MT] x :%4d, y :%4d, z :%4d, finger:%4d, width:%4d\n", x, y, z, finger, width);
				input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, z);
				input_mt_sync(melfas_ts->input_dev);

				if(do_report2) {
					debugprintk(5,"[ABS_MT] x2:%4d, y2:%4d, z2:%4d, finger:%4d, width:%4d\n", x2, y2, z2, finger, width);
					input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, x2);
					input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, y2);
					input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, z2);
					input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, z2);
					input_mt_sync(melfas_ts->input_dev);
				}
			}
			else {
	            debugprintk(5,"[TOUCH_ABS] x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);
	            debugprintk(5," x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);
	            if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
	            if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);

	            input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
	            input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
	            input_report_key(melfas_ts->input_dev, BTN_TOUCH, finger);
			}
			#else
            debugprintk(5,"[TOUCH_ABS] x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);
            debugprintk(5," x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);
            if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
            if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);

            input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
            input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
            input_report_key(melfas_ts->input_dev, BTN_TOUCH, finger);
			#endif
        }
    }

    input_sync(melfas_ts->input_dev);
  }
  enable_irq(melfas_ts->irq);
}

irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq(melfas_ts->irq);
	queue_work(melfas_ts_wq, &melfas_ts->work);
	return IRQ_HANDLED;
}

//int melfas_ts_probe(struct i2c_client *client)
int melfas_ts_probe()
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	int fw_ver = 0;
	int hw_rev = 0;

	if (!i2c_check_functionality(melfas_ts->client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	INIT_WORK(&melfas_ts->work, melfas_ts_work_func);

#if 0
    ret = i2c_smbus_write_byte_data(melfas_ts->client, 0x01, 0x01); /* device command = reset */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
	}
#endif
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, 0x20);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		//this is not critical.. let's go on... goto err_detect_failed;
	}
	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, 0x1E);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}

	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x\n", fw_ver);
	printk(KERN_INFO "melfas_ts_probe: Hardware Revsion %x\n", hw_rev);

	melfas_ts->fw_ver = fw_ver;
	melfas_ts->hw_rev = hw_rev;
	//melfas_ts->irq = IRQ_TOUCH_INT;

	melfas_check_mt_support(1);

	ret = i2c_smbus_read_word_data(melfas_ts->client, 0x08);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		//goto err_detect_failed;
	}
	else {
		max_x = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	}
	ret = i2c_smbus_read_word_data(melfas_ts->client, 0x0a);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		//goto err_detect_failed;
	}
	else {
		max_y = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	}

#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_MAX)
	if(max_x == 0 || max_y == 0) {
		max_x = 240;
		max_y = 400;
	}
#elif defined(CONFIG_MACH_VITAL)
	if(max_x == 0 || max_y == 0) {
		max_x = 320;
		max_y = 480;
	}
#endif

	melfas_ts->input_dev = input_allocate_device();
	if (melfas_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_ts->input_dev->name = "melfas_ts_input";

	set_bit(EV_SYN, melfas_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_ts->input_dev->evbit);
	set_bit(TOUCH_HOME, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_ts->input_dev->keybit);
#if defined(CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_MAX) || defined(CONFIG_MACH_VITAL)
      set_bit(TOUCH_SEARCH, melfas_ts->input_dev->keybit);
#endif

	melfas_ts->input_dev->keycode = melfas_ts_tk_keycode;
	set_bit(BTN_TOUCH, melfas_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_ts->input_dev->evbit);

	melfas_ts->info.max_x = max_x;
	melfas_ts->info.max_y = max_y;

	#ifdef USES_PINCH_DIST_MT

	if(melfas_ts->mt_support == true) {
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	}
	else {
		input_set_abs_params(melfas_ts->input_dev, ABS_X, 0, max_x, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_Y, 0, max_y, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	}
	#else
	input_set_abs_params(melfas_ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	#endif

	printk("melfas_ts_probe: max_x %d, max_y %d\n", max_x, max_y);

	/* melfas_ts->input_dev->name = melfas_ts->keypad_info->name; */
	ret = input_register_device(melfas_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", melfas_ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ret = request_irq(melfas_ts->client->irq, melfas_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	if(ret == 0) {
		printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s \n", melfas_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_ts->early_suspend.suspend = melfas_ts_early_suspend;
	melfas_ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	return 0;

err_input_register_device_failed:
	input_free_device(melfas_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
err_check_functionality_failed:
	return ret;
}

int melfas_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_ts->irq, 0);
	input_unregister_device(melfas_ts->input_dev);
	return 0;
}


int melfas_ts_gen_touch_up(void)
{
    // report up key if needed
    if(melfas_ts->info.state == 0x1) //down state
    {
        melfas_ts->info.state = 0x0;
        int x = melfas_ts->info.x;
        int y = melfas_ts->info.y;
        int z = melfas_ts->info.z;
        int width = melfas_ts->info.width;
        printk("[TSP] GENERATE UP KEY x: %4d, y: %4d, z: %4d\n", x, y, z);
        if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
        if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);

        input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
        input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
        input_report_key(melfas_ts->input_dev, BTN_TOUCH, 0);

        input_sync(melfas_ts->input_dev);
    }
}
//int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
int melfas_ts_suspend(pm_message_t mesg)
{
    melfas_ts->suspended = true;
    melfas_ts_gen_touch_up();
	disable_irq(melfas_ts->irq);
    gpio_set_value(GPIO_TOUCH_EN, 0);  // TOUCH EN

	return 0;
}

//int melfas_ts_resume(struct i2c_client *client)
int melfas_ts_resume()
{
    gpio_set_value(GPIO_TOUCH_EN, 1);  // TOUCH EN
	msleep(300);
    melfas_ts->suspended = false;
	enable_irq(melfas_ts->irq);

	return 0;
}


int tsp_preprocess_suspend(void)
{
#if 0 // blocked for now.. we will gen touch when suspend func is called
    // this function is called before kernel calls suspend functions
    // so we are going suspended if suspended==false
    if(melfas_ts->suspended == false) {

        // fake as suspended
        melfas_ts->suspended = true;

        //generate and report touch event
        melfas_ts_gen_touch_up();
    }
#endif
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h)
{
	melfas_ts_suspend(PMSG_SUSPEND);
}

void melfas_ts_late_resume(struct early_suspend *h)
{
	melfas_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

int melfas_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_ts->client = client;
	i2c_set_clientdata(client, melfas_ts);
	return 0;
}

static int __devexit melfas_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&melfas_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
    free_irq(melfas_ts->client->irq, 0);
    input_unregister_device(melfas_ts->input_dev);

   	melfas_ts = i2c_get_clientdata(client);
	kfree(melfas_ts);
    return 0;
}

struct i2c_device_id melfas_id[] = {
	{ "melfas_ts_i2c", 0 },
	{ }
};

struct i2c_driver melfas_ts_i2c = {
	.driver = {
		.name	= "melfas_ts_i2c",
		.owner	= THIS_MODULE,
	},
	.probe 		= melfas_i2c_probe,
	.remove		= __devexit_p(melfas_i2c_remove),
	.id_table	= melfas_id,
};

void init_hw_setting(void)
{

	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(GPIO_TOUCH_INT_AF));
	s3c_gpio_setpull(GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);

	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_EDGE_FALLING);
//	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW);

	if (gpio_is_valid(GPIO_TOUCH_EN)) {
		if (gpio_request(GPIO_TOUCH_EN, S3C_GPIO_LAVEL(GPIO_TOUCH_EN)))
			printk(KERN_ERR "Filed to request GPIO_TOUCH_EN!\n");
		gpio_direction_output(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
	}
//mk93.lee Phone power on prob.	s3c_gpio_setpull(GPIO_PHONE_ON, S3C_GPIO_PULL_UP);
}

struct platform_driver melfas_ts_driver =  {
	.probe	= melfas_ts_probe,
	.remove = melfas_ts_remove,
	.driver = {
		.name = "melfas-ts",
		.owner	= THIS_MODULE,
	},
};


int __init melfas_ts_init(void)
{
	int ret;

	init_hw_setting();

    gpio_set_value(GPIO_TOUCH_EN, 1);  // TOUCH EN
	mdelay(300);

	ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
#ifdef USES_PINCH_DIST_MT
	if (device_create_file(ts_dev, &dev_attr_multitouch) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_multitouch.attr.name);
#endif
#if defined(CONFIG_MACH_VINSQ)
	if (device_create_file(ts_dev, &dev_attr_enhance_oj) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_enhance_oj.attr.name);
#endif

	melfas_ts = kzalloc(sizeof(struct melfas_ts_driver), GFP_KERNEL);
	if(melfas_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&melfas_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	if(!melfas_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&melfas_ts_i2c);
		return 0;
	}
	melfas_ts_wq = create_singlethread_workqueue("melfas_ts_wq");
	if (!melfas_ts_wq)
		return -ENOMEM;

	return platform_driver_register(&melfas_ts_driver);

}

void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_i2c);
	if (melfas_ts_wq)
		destroy_workqueue(melfas_ts_wq);
}
late_initcall(melfas_ts_init);
//module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
