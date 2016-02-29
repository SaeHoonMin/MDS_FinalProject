/**
 @project	
 		G2450_PWM Device Driver Header 
 	
 		2014.01.17  by Lee Dong su
 
 @section intro
		 	
 @section Program 
 	 	
 @section MODIFYINFO 
 
 
*/

#ifndef __G2450_PWM_H__
#define __G2450_PWM_H__

struct g2450_pwm_duty_t {
	int pulse_width;		// nsec
	int period;				// nsec
};

#define DEV_PWM_MAJOR	220
#define DEV_PWM_NAME	"g2450_pwm"

#define DEV_PWM_IOCTL_MAGIC	'p'

#define DEV_PWM_RUN			_IO(DEV_PWM_IOCTL_MAGIC, 0		)
#define DEV_PWM_STOP		_IO(DEV_PWM_IOCTL_MAGIC, 1 		)
#define DEV_PWM_DUTYRATE	_IOW(DEV_PWM_IOCTL_MAGIC, 2, struct g2450_pwm_duty_t )
#define DEV_PWM_IOCTL_MAXNR	3

#endif //__G2450_PWM_H__

