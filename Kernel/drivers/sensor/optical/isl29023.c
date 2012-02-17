/* 
 *  Title : Optical Sensor driver for ISL29023 light sensor 
 *  Date  : 14.10.2011
 *  Name  : Sandor Bognar
 *
 */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <asm/uaccess.h>

#include <linux/input.h>
#include <linux/workqueue.h>

#include "isl29023.h"

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* global var */
static struct i2c_client *isl_i2c_client = NULL;
struct light_data *light;

static int lux_value = 0;
static int screen_correction = 10; 
static int print_log = 0;
static u16 op_mode = 0x2000;
static bool light_enable = ON;

static unsigned short isl_normal_i2c[] = {(LIGHT_ADDR>>1),I2C_CLIENT_END};
static unsigned short isl_ignore[] = {1,(LIGHT_ADDR>>1),I2C_CLIENT_END};
static unsigned short isl_probe[] = {I2C_CLIENT_END};

static struct i2c_client_address_data isl_addr_data = {
	.normal_i2c = isl_normal_i2c,
	.ignore		= isl_ignore,
	.probe		= isl_probe,	
};


static int __devinit isl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *);
static int __devexit isl_i2c_remove(struct i2c_client *);
static int isl_i2c_detect(struct i2c_client *, int kind, struct i2c_board_info *);

static struct i2c_device_id isl_i2c_id[] = {
	{ "ISL29023", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, isl_i2c_id);

static struct i2c_driver isl_i2c_driver = {
	.driver = {
		.name = "ISL29023",
	},
	.class 		= I2C_CLASS_HWMON,
	.probe		= isl_i2c_probe,
	.remove		= __devexit_p(isl_i2c_remove),
	.detect		= isl_i2c_detect,
	.id_table	= isl_i2c_id,
	.address_data	= &isl_addr_data,
};

#if 0
void print_sensor_regs(void) {
	gprintk("REGS_COMMAND_I: %x\n", isl_i2c_read((u8)(REGS_COMMAND_I)));
	gprintk("REGS_COMMAND_II: %x\n", isl_i2c_read((u8)(REGS_COMMAND_II)));
	gprintk("REGS_INT_MSB_TH_LO: %x\n", isl_i2c_read((u8)(REGS_INT_MSB_TH_LO)));
	gprintk("REGS_INT_LSB_TH_LO: %x\n", isl_i2c_read((u8)(REGS_INT_LSB_TH_LO)));
	gprintk("REGS_INT_MSB_TH_HI: %x\n", isl_i2c_read((u8)(REGS_INT_MSB_TH_HI)));
	gprintk("REGS_INT_LSB_TH_HI: %x\n", isl_i2c_read((u8)(REGS_INT_LSB_TH_HI)));
	gprintk("REGS_MBS_SENSOR: %x\n", isl_i2c_read((u8)(REGS_MBS_SENSOR)));
	gprintk("REGS_LBS_SENSOR: %x\n", isl_i2c_read((u8)(REGS_LBS_SENSOR)));
}	
#endif

int adc_to_lux(int value) {

	// see isl29023.h for detailes
	return FSR_LUX_range[op_mode & 3] * value/ 
			adc_resolution[(op_mode >> 2) & 3];
}

/*****************************************************************************************
 *  
 *  function    : work_func_light 
 */
void work_func_light(struct work_struct *work) {

	/* read light data from sensor i2c */
	unsigned char v_msb;
	unsigned char v_lsb;
	unsigned int vout = 0;
	int range;

re_messure:	
	isl_i2c_write((u8)(REGS_COMMAND_I), (op_mode >> 8) & 0xff); 
	isl_i2c_write((u8)(REGS_COMMAND_II), op_mode & 0xff);

	//print_sensor_regs();

	v_msb = isl_i2c_read((u8)(REGS_MBS_SENSOR));
	v_lsb = isl_i2c_read((u8)(REGS_LBS_SENSOR));
	vout =  (v_msb << 8) | v_lsb;

	/* see isl29023.h for detailes */
	range = op_mode & 3;
	lux_value = FSR_LUX_range[range] * vout/ 
			adc_resolution[(op_mode >> 2) & 3];

	if (lux_value >= FSR_LUX_range[range]) { 
		if ((range) < 3) {
			if (print_log) 	
				printk("OP_MODE: %x ADC= %d LUX= %d\n", 
					op_mode, vout, lux_value);
		       	op_mode++; 
			goto re_messure; 
		}
	} 
	if ((range) > 0) { 
		if (lux_value < FSR_LUX_range[range - 1]){	
			if (print_log) 	
				printk("OP_MODE: %x ADC= %d LUX= %d\n", 
					op_mode, vout, lux_value);
			op_mode--; 
			goto re_messure; 
		}	
	}	

	/* screen_correction is to compensate the light loss caused by sensor's window) */
	input_report_abs(light->input_dev,ABS_MISC, lux_value * screen_correction);
	input_sync(light->input_dev);
	mdelay(1);

	if (print_log) 	
		printk("Ligh Sensor OP_MODE: %x ADC= %d LUX= %d\n", op_mode, vout, lux_value * screen_correction);
}


static enum hrtimer_restart light_timer_func(struct hrtimer *timer) {
	
	queue_work(light_wq, &light->work_light);
	hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD, 0),HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int isl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret,err=0;
	
	gprintk("\n");
	if ( !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		return err;
	}

	client->addr = LIGHT_ADDR >> 1;
	client->driver = &isl_i2c_driver;
	client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	isl_i2c_client = client;

	gprintk("\n");

	printk("[ISL29023] COMPLETE I2C PROBE\n");
	return ret;

}

static int __devexit isl_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int isl_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strlcpy(info->type, "ISL29023", I2C_NAME_SIZE);
	return 0;
}

static int isl_i2c_init(void) {
	if( i2c_add_driver(&isl_i2c_driver)) {
		printk("i2c_add_driver failed \n");
		return -ENODEV;
	}
	return 0;
}

int isl_i2c_read(u8 reg) {
	int val;
	val = i2c_smbus_read_byte_data(isl_i2c_client, reg);	
	if (val < 0)
		printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return val;
}

int isl_i2c_write( u8 reg, int val ) {
	int err;

	if( (isl_i2c_client == NULL) || (!isl_i2c_client->adapter) ){
		return -ENODEV;
	}
	err = i2c_smbus_write_byte_data(isl_i2c_client, reg, val);
	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

void light_chip_init(void) {
	gprintk("\n");
	
	// to disable chip irq pin
	if (gpio_is_valid(GPIO_AMBIENT_INT_N)) {
		if (gpio_request(GPIO_AMBIENT_INT_N, S3C_GPIO_LAVEL(GPIO_AMBIENT_INT_N)))
			printk(KERN_ERR "Failed to request GPIO_AMBIENT_INT_N!\n");
		gpio_direction_output(GPIO_AMBIENT_INT_N, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(GPIO_AMBIENT_INT_N, S3C_GPIO_PULL_NONE);
	
	}	
}

void light_on(void) {
	if( light_enable == OFF) {
		gprintk("light power on \n");
		gprintk("timer start for light sensor\n");
		hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD,0),HRTIMER_MODE_REL);
	}
}

void light_off(void) {
	if(light_enable == ON) {
		gprintk("Light sensor power off \n");

		gprintk("timer cancel for light sensor\n");
		hrtimer_cancel(&light->timer);
		
		isl_i2c_write((u8)(REGS_COMMAND_I), 0x0);
		isl_i2c_write((u8)(REGS_COMMAND_II), 0x0);

		mdelay(5);
	}
}

/* MISC device operations */
static int isl_light_open(struct inode *ip, struct file *fp) {
	return 0;
}

static int isl_light_release(struct inode *ip, struct file *fp) {
	return 0;
}

static long isl_light_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	int ret = 0;
	int value = 0;

	void __user *argp = (void __user *)arg;

	switch(cmd) {
		case KIONIX_ISL_IOCTL_ENABLE:
			if (copy_from_user(&value, argp, sizeof(value)))
				return -EFAULT;

			printk(KERN_INFO "[LIGHT_SENSOR] %s : case ENABLE: %d\n", __FUNCTION__, value);
			if(value == ON && light_enable == OFF) {
				light_on();
				light_enable = ON;
			}
			else if(value == OFF && light_enable ==ON) {
				light_off();
				light_enable = OFF;
			}
			break;

		case KIONIX_ISL_IOCTL_GET_ENABLED:
			value = light_enable;
			if (copy_to_user(argp, &value, sizeof(value)))	
				return -EFAULT;
			break;

		case KIONIX_ISL_PRINT_ON:
			print_log = 1;
			printk("KIONIX_ISL_PRINT_ON\n");
			break;

		case KIONIX_ISL_PRINT_OFF:
			print_log = 0;
			printk("KIONIX_ISL_PRINT_OFF\n");
			break;

		default:
			printk(KERN_INFO "[LIGHT_SENSOR] unknown ioctl %d\n", cmd);
			ret = -1;
			break;
	}
	return ret;
}

static struct file_operations isl_light_fops = {
	.owner  = THIS_MODULE,
	.open   = isl_light_open,
	.release = isl_light_release,
	.unlocked_ioctl = isl_light_ioctl,
};
                 
static struct miscdevice isl_light_device = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = "lightsensor",
	.fops   = &isl_light_fops,
};

static int isl_light_probe( struct platform_device* pdev ) {
	int ret;

	/* allocate driver_data */
	light = kzalloc(sizeof(struct light_data),GFP_KERNEL);
	if(!light) {
		pr_err("kzalloc error\n");
		return -ENOMEM;
	}
	gprintk("in %s \n",__func__);
	
	/* init i2c */
	isl_i2c_init();

	if(isl_i2c_client == NULL) {
		pr_err("isl_probe failed : i2c_client is NULL\n"); 
		return -ENODEV;
	}

	/* hrtimer Settings */
	hrtimer_init(&light->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	light->timer.function = light_timer_func;

	/* WORK QUEUE Settings */
    	light_wq = create_singlethread_workqueue("light_wq");
	if (!light_wq)
	    return -ENOMEM;
        INIT_WORK(&light->work_light, work_func_light);
	gprintk("Workqueue Settings complete\n");

	/* Input device Settings */
	light->input_dev = input_allocate_device();
	if (light->input_dev == NULL) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}
	light->input_dev->name = "LightSensor";

	set_bit(EV_SYN,light->input_dev->evbit);
	set_bit(EV_ABS,light->input_dev->evbit);
	
	input_set_abs_params(light->input_dev, ABS_MISC, 0, 1, 0, 0);

	ret = input_register_device(light->input_dev);
	if (ret) {
		pr_err("Unable to register %s input device\n", light->input_dev->name);
		input_free_device(light->input_dev);
		kfree(light);
		return -1;
	}

	/* misc device Settings */
	ret = misc_register(&isl_light_device);
	if(ret) {
		pr_err(KERN_ERR "misc_register failed \n");
	}

	light_off();
	light_enable = OFF;

	printk("ISL29023 Light Sensor initialized\n");
	
	return 0;
}

static int isl_light_suspend( struct platform_device* pdev, pm_message_t state ) {

	if(light_enable) {
		isl_i2c_write((u8)(REGS_COMMAND_I), 0x0);
		isl_i2c_write((u8)(REGS_COMMAND_II), 0x0);
	}	
	return 0;
}


static int isl_light_resume( struct platform_device* pdev ) {

	mdelay(5);
	
	if(light_enable) {
		hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD,0),HRTIMER_MODE_REL);
	}
	return 0;
}

static struct platform_driver isl_light_driver = {
	.probe 	 = isl_light_probe,
	.suspend = isl_light_suspend,
	.resume  = isl_light_resume,
	.driver  = {
		.name = "isl29023-opt",
		.owner = THIS_MODULE,
	},
};

static int __init isl_light_init(void) {
	int ret;
	
	light_chip_init();
	ret = platform_driver_register(&isl_light_driver);
	return ret;
}

static void __exit isl_light_exit(void) {
	if (light_wq)
		destroy_workqueue(light_wq);

	input_unregister_device(light->input_dev);
	kfree(light);

	gpio_free(GPIO_AMBIENT_INT_N);
	platform_driver_unregister(&isl_light_driver);
}

module_init( isl_light_init );
module_exit( isl_light_exit );

MODULE_AUTHOR("BS");
MODULE_DESCRIPTION("Optical Sensor driver for ISL29023");
MODULE_LICENSE("GPL");
