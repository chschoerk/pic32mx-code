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
                //if (tmpDebug == 200){
                //    tmpDebug = 0;
                //}
            }
            edgeCount -= thisVal;
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



