#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "measuring.h"
#include "pwm_vcxo.h"

/*
 * setupPWM
 */
UINT32 setupPWM(unsigned int fpb)
{
    UINT32 pr2;
    unsigned int prescalar = 1;
    
    pr2 = (UINT32)((fpb/(PMW_FREQUENCY*prescalar)) - 1);
    /*PWM resolution [bits] = log2(peripheral bus frequency / (PWM frequency*prescalar)
     *e.g. fpb = 40MHz, PWM_FREQUENCY = 100Hz, prescalar = 1 ==> resolution = 18.6 bits
     *INFO: The lower PWM frequency, the highter the PWM resolution - BUT: The lower the
     *PWM frequency, the higher the ripple after the low-pass filter .
     */

    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE | OC_TIMER_MODE32, 0, 0);
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT | T2_32BIT_MODE_ON, pr2);
    SetDCOC1PWM(pr2/2); //50% duty cycle for starters
    return (pr2/2);
}

int updateDutyCycle(UINT32 dutyCycle)
{
    return 0;
}

int setupEdgeCount()
{
    OpenTimer1(T1_ON | T1_SOURCE_EXT | T1_PS_1_1, T1PR); //no prescalor other than 1_1 work's?!
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);
    return 0;
}

