/* 
 * File:   pwm_vcxo.h
 * Author: Administrator
 *
 * Created on 18. März 2013, 13:00
 */

#ifndef PWM_VCXO_H
#define	PWM_VCXO_H

#ifdef	__cplusplus
extern "C" {
#endif

#define PMW_FREQUENCY   100

UINT32 setupPWM(unsigned int fpb);
int updateDutyCycle(UINT32 dutyCycle);
int setupEdgeCount(void);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_VCXO_H */

