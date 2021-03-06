/*
 * Header for Kionix KXSD9-2042 accelerometer.
 */
#ifndef __KXSD9_2042_H__
#define __KXSD9_2042_H__


// Address for KXSD9 embedded registers
#define KXSD9_REG_XOUT_H	0x00  //R
#define KXSD9_REG_XOUT_L	0x01  //R
#define KXSD9_REG_YOUT_H	0x02  //R
#define KXSD9_REG_YOUT_L	0x03  //R
#define KXSD9_REG_ZOUT_H	0x04  //R
#define KXSD9_REG_ZOUT_L	0x05  //R
#define KXSD9_REG_AUXOUT_H	0x06  //R
#define KXSD9_REG_AUXOUT_L	0x07  //R

#define KXSD9_REG_RESET_WRITE	0x0A  //W

#define KXSD9_REG_CTRL_REGC	0x0C  //R/W
#define KXSD9_REG_CTRL_REGB	0x0D  //R/W
#define KXSD9_REG_CTRL_REGA	0x0E  //R

#define KSXD9_VAL_RESET	0xCA

#define RBUFF_SIZE		31	/* Rx buffer size */

#define ACCS_IOCTL_OPEN 	101
#define ACCS_IOCTL_CLOSE 	102

/*********** for debug **********************************************************/
#if 0
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif 
/*******************************************************************************/

#define KXSD9_TESTMODE
#define KXSD9_TESTMODE_PERIOD 500000000  // 0.5 sec
//#define KXSD9_TESTMODE_PERIOD 1000000000  // 1 sec

#define KXSD9_MODE_NORMAL      0
#define KXSD9_MODE_SLEEP       2
#define KXSD9_MODE_WAKE_UP     3



typedef struct  {
	                short  x, y, z; 
} bma020acc_t;

bma020acc_t acc_data;

#endif   // __KXSD9_2042_H__

