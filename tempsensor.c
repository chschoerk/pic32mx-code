#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "tempsensor.h"

#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_12


int setupTempSensor()
{

    CloseADC10();
    SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 | ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10(PARAM1, PARAM2, PARAM3, ENABLE_AN0_ANA, SKIP_SCAN_ALL);
    EnableADC10();

    return 0;
}

UINT8 readTemperature()
{
    unsigned int ADCbuffoffset; // points to the base of the idle buffer
    unsigned int resultADC;
    INT32 temperature = 0;

    ADCbuffoffset = 8 * ((~ReadActiveBufferADC10() & 0x01));
    resultADC = ReadADC10(ADCbuffoffset);

    /*T = ((3.3*ADCvalue/1024) - 0.4V) / 0.0195V. Refer to datasheet of sensor MCP9701A*/
    temperature = (INT32)(165*resultADC) - 20500;

    /*limit to -10 to + 118 Celsius*/
    if (temperature < -10000)
        temperature = -10000;
    if (temperature > 118000)
        temperature = 118000;

    /*squeeze to unsigned 8 bit range*/
    temperature = 2*(temperature + 10000);
    temperature = temperature/1000; //the temperature in Celsius is computed with [C] = (temperature/2)-10

    return temperature;
}