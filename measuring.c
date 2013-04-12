#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
//#include <string.h>

#include "measuring.h"


//UINT32 txCounterPeriod = T1TURNS * T1PR;
//UINT32 txCounterPeriod = 3931904;
extern volatile UINT32 nominalValue;

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

    if (turns == 0 || turns > MAX_MISSED_FOR_SANITY){
        sanity = 0;
        return sanity;
    }

    sanityThresh = REFEDGES >> 10; //REFEDGES / 1024
    v1 = edgeCount/turns;

    if (v1 > REFEDGES){
        dist = v1 - REFEDGES;
    }else{
        dist = REFEDGES - v1;
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
            indivError = (INT32)(thisVal - nominalValue); //fill buffer with individual errors

            //DEBUG
            /*if ( (indivError < -2000) || (indivError > 2000) ){
                indivError++;
            }*/

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
                /*TRY: return an estimated error, as if the buffer was already full*/
                /*
                if (bfIdx > 0){ 
                    *pError = TMEAS_BUFFER_SIZE * (*pBufSum); //TODO secure this operation (OVERFLOW!)
                    *pError = *pError / bfIdx;
                } else {
                    *pError = *pBufSum;
                }*/
                        
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

int limitSigned(INT32 *in, INT32 plim, INT32 nlim)
{
    int ret = 0;

    if ((*in) > plim){
        (*in) = plim;
        ret = 1;
    }else if ((*in) < nlim){
        (*in) = nlim;
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

INT32 secureAdd_INT32(INT32 a, INT32 b)
{
    INT32 max = 0x7fffffff;
    INT32 min = -max-1;

    if (a < 0 && b < 0){
        if ( (min-a) > -b ){
            return (a+b);
        }else{
            return min;
        }

    }else if (a > 0 && b > 0){
        if ( (max-a) > b){
            return (a+b);
        }else{
            return max;
        }

    }else{
        return (a+b);
    }

}

INT32 PID(INT32 error, UINT32 minPWMval, UINT32 maxPWMval)
{
    INT32 e;
    static INT32 eOld = 0;
    static INT32 x_int = 0;
    static int sat = 0;
    INT32 x_diff = 0;
    INT32 x_prop = 0;
    INT32 x = 0;
   
    INT32 plimS19 = (1<<18)-1;
    INT32 nlimS19 =  -(1<<18);

    e = error;

    if (0){
    //if ((sat < 0 && e < 0) || (sat > 0 && e > 0)){ //anti-windup
        /* do nothing if there is saturation, and error is in the same direction;
         * if you're careful you can implement as "if (sat*e > 0)"
         */
        //return -1;
    }else{

        limitSigned(&e, plimS19, nlimS19); //limit to 19 bit signed

        /*integral part*/
        if (sat == 0){
            x_int = secureAdd_INT32(x_int, KI*e);  //x_int = x_int + KI*e;
        }
        //sat = limitSigned(&x_int, plimS19, nlimS19);

        /*differental part
        x_diff = e - eOld; //need security?
        limitSigned(&x_diff, plimS19, nlimS19);
        x_diff *= KD;
        x_diff = x_diff >> 12;
        limitSigned(&x_diff, plimS19, nlimS19);
        */

        /*proportional part*/
        x_prop = KP * e; //e is limited to signed 19 bit, if KP is < 8192 no problem
        limitSigned(&x_prop, plimS19, nlimS19);

        /*controller equation*/
        x = (x_prop>>SCAL) + (x_int>>SCAL) + x_diff;

        /*scale to output (pwm) format [signed to unsigned]*/
        x += ( (maxPWMval-minPWMval) >>1 );

        sat = limitSigned(&x, maxPWMval, minPWMval); //pr2 = (UINT32)((fpb/(PMW_FREQUENCY*prescalar)) - 1); (vgl. setupEdgeCount)

        eOld = e;

        return x;
    }
    
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



