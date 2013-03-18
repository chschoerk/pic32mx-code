#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "pwm_vcxo.h"



int setupPWM()
{
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE | OC_TIMER_MODE16, 0, 0);
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
    SetDCOC1PWM(0x7FFF); //50% duty cycle
    return 0;
}