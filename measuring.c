#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
//#include <string.h>

#include "measuring.h"


UINT32 txCounterPeriod = T1TURNS * T1PR;



int setupDetectInterrupt()
{
    INTSetVectorPriority(INT_VECTOR_EX_INT(1), INT_PRIORITY_LEVEL_3); //set INT controller priority
    INTSetVectorSubPriority(INT_VECTOR_EX_INT(1), INT_SUB_PRIORITY_LEVEL_3); //set INT controller sub-priority
    INTCONbits.INT1EP = 1; //edge polarity -> rising edge
    IFS0 &= ~0x0100; //clear interrupt flag
    IEC0 |= 0x0100; //enable INT1 interrupt
    return 0;
}


/*function: updateBuffer*/
BOOL updateBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx)
{
    *bufSum = *bufSum - buf[*bfIdx] + val;
    buf[*bfIdx] = val;
    (*bfIdx)++;
    *bfIdx &= (TMEAS_BUFFER_SIZE-1); //equals: bfIdx = bfIdx % TMEAS_BUFFER_SIZE if TMEAS_BUFFER_SIZE is 2^x

    return 0;
}

BOOL fillBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx)
{
    BOOL full = FALSE;

    *bufSum += val;
    buf[*bfIdx] = val;
    (*bfIdx)++;
    if (*bfIdx == TMEAS_BUFFER_SIZE){
        full = TRUE;
        *bfIdx = 0;
    }
    return full;
}

int sanityCheck(UINT32 edgeCount, UINT32 turns)
{
    UINT32 v1, dist, sanityThresh;
    int sanity = 0;

    if (turns == 0){
        sanity = 0;
        return sanity;
    }

    sanityThresh = txCounterPeriod >> 10; //txCounterPeriod / 1024
    v1 = edgeCount/turns;

    if (v1 > txCounterPeriod){
        dist = v1 - txCounterPeriod;
    }else{
        dist = txCounterPeriod - v1;
    }

    if (dist < sanityThresh){
        sanity = 1;
    }

    return sanity;
}

int measureFrequency(UINT32 edgeCount, INT32 *buf,
                     INT32 *pBufSum, UINT32 turns, INT32 *pError)
{
    UINT32 thisVal;
    UINT32 i;
    INT32  indivError;
    static BOOL bufferFull = FALSE;
    static unsigned int bfIdx=0;
    int ret;
    static int tmpDebug = 0;

    
    /*Fill buffer with value(s). If packets have been skipped,
     *distribute values equally across buffer entries (no rounding)*/
    for (i = turns; i > 0; i--){
            thisVal = edgeCount/i;
            indivError = (INT32)(thisVal - txCounterPeriod); //fill buffer with individual errors

            //DEBUG
            if ( (indivError < -2000) || (indivError > 2000) ){
                indivError++;
            }

            if (bufferFull){
                updateBuffer(indivError, pBufSum, buf, &bfIdx);
                /*estimate frequency error [ppb]*/
                //tmp_i1 = *pBufSum - bufSumRef; //signed int!
                //tmp_i1 = *pBufSum;
                //tmp_f1 = (float)1000000 * (float)tmp_i1;
                //*pError = tmp_f1 / (float)bufSumRef;
                *pError = *pBufSum;
                ret = 1;
            }else{
                bufferFull = fillBuffer(indivError, pBufSum, buf, &bfIdx);
                *pError = *pBufSum;
                ret = 0; //not yet full
                //tmpDebug++;
                //if (tmpDebug == 100){
                //    tmpDebug = 0;
                //}
            }
            edgeCount -= thisVal;
    }
       
   return ret;
}

int limitSigned(INT32 *in, INT32 lim)
{
    int ret = 0;

    if ((*in) > (lim-1)){
        (*in) = lim-1;
        ret = 1;
    }else if ((*in) < -lim){
        (*in) = -lim;
        ret = -1;
    }

    return ret;
}

int limitUnsigned(UINT32 *in, UINT32 lim)
{
    int ret = 0;

    if ((*in) > lim){
        (*in) = lim;
        ret = 1;
    }

    return ret;
}

INT32 PID(INT32 error)
{
    INT32 kp, ki, kd;
    INT32 e;
    static INT32 eOld = 0;
    static INT32 x_int = 0;
    INT32 x_diff = 0;
    INT32 x_prop = 0;
    INT32 x = 0;
    static int sat = 0;
    INT32 out = 0;

    e = error;

    if ((sat < 0 && e < 0) || (sat > 0 && e > 0)){ //anti-windup
        /* do nothing if there is saturation, and error is in the same direction;
         * if you're careful you can implement as "if (sat*e > 0)"
         */
    }else{

        e = limitSigned(&e, 1<<18); //limit to 19 bit signed



        /*integral part*/
        x_int = x_int + KI*e;
        sat = limitSigned(&x_int, LIM_INT);

        /*differental part*/
        x_diff = e - eOld;
        limitSigned(&x_diff, LIM_INT);
        x_diff *= kd;
        x_diff = x_diff >> 12;
        limitSigned(&x_diff, LIM_INT);

        /*proportional part*/
        x_prop = kp * e;
        limitSigned(&x_prop, LIM_INT);

        /*controller equation*/
        x = x_prop + x_int + x_diff;
        //limitUnsigned(x, 399999); //pr2 = (UINT32)((fpb/(PMW_FREQUENCY*prescalar)) - 1); (vgl. setupEdgeCount)
    }

    eOld = error;

    return 0;
}


INT32 anotherFilter(INT32 input)
{
    INT32 output = 0;
    output = input;
    return output;
    /*Additional Filter (?)*/
                    /* e.g. simple low pass filter*/
                    /*
                    tmpFilt = (filtOut >> FILT_SZ);
                    filtOut -= tmpFilt;
                    filtOut += (counterDiff >> FILT_SZ);
                    */

                    /*or moving average filter
                    if (MAWindowFull){
                        maSum = maSum - TimerVals[tarrInd] + counterDiff;
                        TimerVals[tarrInd] = counterDiff;
                        tarrInd++;
                        tarrInd &= ((0x01<<MA_SIZE)-1); //equals: tarrInd = tarrInd % 2^MA_SIZE
                        maOutput = maSum >> MA_SIZE; //devide by 2^MA_SIZE
                    }else{
                        maSum = maSum + counterDiff; //fill MA window
                        TimerVals[tarrInd] = counterDiff;
                        maOutput = maSum >> MA_SIZE; //devide by 2^MA_SIZE
                        tarrInd++;
                        if (tarrInd == (0x01<<MA_SIZE)){
                            MAWindowFull = TRUE;
                        tarrInd = 0;
                        }
                    }*/
}



