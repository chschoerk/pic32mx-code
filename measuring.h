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

#define T1TURNS         64
#define T1PR            61436  //==> packet send interval: T1TURNS * T1PR

//#define TX_COUNTER_PERIOD   3931904 //Transmitter: 604 turns with 61436 timer setting => 64*61436 = 3931904
                                    //empirical: it's not turns*(T1PR+1) but turns*T1PR.
//#define OUTLIER_THRESH      (TX_COUNTER_PERIOD >> 10)
#define TMEAS_BUFFER_SIZE   512   // ==> measuring period = TMEAS_BUFFER_SIZE*TX_COUNTER_PERIOD/VCO_FREQ (e.g. 163.84s);
                                 //caution: largest number on PIC32: 2^32. if buffer is to long the elapsed time between two points is larger than 2^32.
                                 //TMEAS_BUFFER_SIZE GOT TO BE 2^X!
//#define BUF_SUM_REF         (TX_COUNTER_PERIOD * TMEAS_BUFFER_SIZE)
#define LIM_INT 0

#define SCAL 13 //2^13    //DAS MUSS WAHRSCHEINLIHC 12 sein!!!
#define KI 82 // 0.01*2^13 (siehe matlab script)
#define KP 3277 //0.4*2^13
#define KD 0



    
int setupDetectInterrupt(void);
BOOL updateBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx);
BOOL fillBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx);
int sanityCheck(UINT32 edgeCount, UINT32 turns);
INT32 anotherFilter(INT32 input);
int measureFrequency(UINT32 edgeCount, INT32 *buf,
                     INT32 *pBufSum, UINT32 turns, INT32 *pError);
INT32 PID(INT32 error);
int limitUnsigned(UINT32 *in, UINT32 lim);
int limitSigned(INT32 *in, INT32 lim);


#ifdef	__cplusplus
}
#endif

#endif	/* MEASURING_H */

