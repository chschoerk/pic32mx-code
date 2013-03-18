/* 
 * File:   measuring.h
 * Author: Administrator
 *
 * Created on 18. März 2013, 13:51
 */

#ifndef MEASURING_H
#define	MEASURING_H

#ifdef	__cplusplus
extern "C" {
#endif


#define TX_COUNTER_PERIOD   3932100 //Transmitter: 60 turns with 0xFFFF(+1) timer setting => 60*65536 = 3932160
                                    //empirical: it's rather 60 x 65535 (i.e. 0xFFFF not 0xFFF+1) ==> 3932100
                                    //this was found by applying the SAME clock source (f-generator) to TX and RX.
                                    //If TX_COUNTER_PERIOD = 3932160 then the error for each time measure between two IRQs is around 60.
#define OUTLIER_THRESH      (TX_COUNTER_PERIOD >> 10)
#define TMEAS_BUFFER_SIZE   512   // ==> measuring period = TMEAS_BUFFER_SIZE*TX_COUNTER_PERIOD/VCO_FREQ (e.g. 163.84s);
                                 //caution: largest number on PIC32: 2^32. if buffer is to long the elapsed time between two points is larger than 2^32.
                                 //TMEAS_BUFFER_SIZE GOT TO BE 2^X!
#define BUF_SUM_REF         (TX_COUNTER_PERIOD * TMEAS_BUFFER_SIZE)



    
int setupDetectInterrupt(void);
BOOL updateBuffer(UINT32 val, UINT32 *bufSum, UINT32 *buf, int *bfIdx);
BOOL fillBuffer(UINT32 val, UINT32 *bufSum, UINT32 *buf, int *bfIdx);
float anotherFilter(float input);
int measureFrequency(unsigned int cntrVal, unsigned int cntrValOld,
                     unsigned int counterOverflow, UINT32 *buf,
                     UINT32 *pBufSum, float *pError);


#ifdef	__cplusplus
}
#endif

#endif	/* MEASURING_H */

