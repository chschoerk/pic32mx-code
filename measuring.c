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


int measureFrequency(unsigned int cntrVal, unsigned int cntrValOld, 
                     unsigned int counterOverflow, INT32 *buf,
                     INT32 *pBufSum, UINT32 turns, INT32 *pError)
{
    UINT32 counterDiff, cDiff, res, thisVal;
    float measTurns_float, tmp_f1;
    UINT32 outlier, i;
    UINT32 outlierThreshold;
    INT32 tmp_i1, indivError;
    static BOOL bufferFull = FALSE;
    static unsigned int bfIdx=0;
    //UINT32 bufSumRef = BUF_SUM_REF;
    int ret;

    outlierThreshold = txCounterPeriod >> 10;

    /*compute counter difference between old value and new value*/
    if (counterOverflow){
        counterDiff = (T1PR - cntrValOld) + ((counterOverflow-1)*T1PR) + cntrVal;
    }else{
        counterDiff = cntrVal - cntrValOld;
    }

    outlier = 0;
    //TODO: dump this ugly computation and use timestamp info instead
    //measTurns_float = counterDiff/(float)txCounterPeriod;
    //measTurns_int = (int)(measTurns_float + 0.5f);
    //if (measTurns_int){
    //    cDiff = counterDiff/measTurns_int; //FLOORED VALUE!
    //}else{
    //    outlier = 1;
    //    measTurns_int = 1; //just so that we don't divide by 0 in the next line
    //}
    cDiff = counterDiff/turns; //FLOORED VALUE!
    res = counterDiff - (cDiff*turns); //residual
    if (res < outlierThreshold && outlier == 0){
        /*Fill buffer with value(s). If packets have been skipped,
         *distribute values equally across buffer entries (no rounding)*/
        for (i = turns; i > 0; i--){
            thisVal = counterDiff/i;
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
            }
            counterDiff -= thisVal;
       }
       

   }else{ //outlier!
        ret = -1; //outlier
        outlier = 0;
   }

   return ret;
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



