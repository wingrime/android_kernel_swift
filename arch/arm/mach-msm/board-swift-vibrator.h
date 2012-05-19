// wingrime 2012 (c) 
// include for swift vibrator
#ifndef __SWIFT_VIBRATOR_
#define __SWIFT_VIBRATOR_

#include <../../../drivers/staging/android/timed_output.h>


/* io memory for vibrator */
#define MSM_WEB_BASE          IOMEM(0xE100C000)
#define MSM_WEB_PHYS          0xA9D00040 
#define MSM_WEB_SIZE          SZ_4K

#define GPIO_VIB_PWM		28

#define GPIO_LDO_EN			109

/* addresses of gp_mn register */
#define GP_MN_CLK_MDIV              0x004C
#define GP_MN_CLK_NDIV              0x0050
#define GP_MN_CLK_DUTY              0x0054

#define GPMN_M_DEFAULT              21
#define GPMN_N_DEFAULT              4500

#define GPMN_D_DEFAULT              3200  /* 2250 */
#define PWM_MULTIPLIER              2560  /* 4394 */

/* The maximum value of M is 511 and N is 8191 */
#define GPMN_M_MASK                 0x01FF
#define GPMN_N_MASK                 0x1FFF
#define GPMN_D_MASK                 0x1FFF

/* Qcoin motor state */
#define PWM_DUTY_MAX		1
#define PWM_DUTY_SET		2
#define PWM_DUTY_MIN		3
#define VIB_DISABLE			4

#define DUTY_MAX_TIME		20
#define DUTY_MIN_TIME		20


 
#define VIB_WRITEL(value, reg)      writel(value, (MSM_WEB_BASE + reg))

struct vibrator_device {
	struct timed_output_dev timed_vibrator;
	struct hrtimer vib_timer;
	int state;
	int value;
	int pwm_max_time;
	int pwm_min_time;
	spinlock_t	lock;
};



#endif

