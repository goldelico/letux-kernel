#ifndef __S3C2410_PWM_H
#define __S3C2410_PWM_H

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm-arm/io.h>
#include <asm/arch/hardware.h>
#include <asm/mach-types.h>
#include <asm/plat-s3c/regs-timer.h>
#include <asm/arch/gta01.h>

enum pwm_timer {
	PWM0,
	PWM1,
	PWM2,
	PWM3,
	PWM4
};

struct s3c2410_pwm {
	enum pwm_timer timerid;
	struct clk *pclk;
	unsigned long pclk_rate;
	unsigned long prescaler;
	unsigned long divider;
	unsigned long counter;
	unsigned long comparer;
};

int s3c2410_pwm_init(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_enable(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_disable(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_start(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_stop(struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_duty_cycle(int reg_value, struct s3c2410_pwm *s3c2410_pwm);
int s3c2410_pwm_dumpregs(void);

#endif /* __S3C2410_PWM_H */
