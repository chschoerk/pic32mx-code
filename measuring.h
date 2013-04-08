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

//#define T1TURNS         64
//#define T1PR            61436  //==> packet send interval: T1TURNS * T1PR
#define T45PR           0xFFFFFFFF
#define REFEDGES        12288000//3931904

//#define TX_COUNTER_PERIOD   3931904 //Transmitter: 604 turns with 61436 timer setting => 64*61436 = 3931904
                                    //empirical: it's not turns*(T1PR+1) but turns*T1PR.
//#define OUTLIER_THRESH      (TX_COUNTER_PERIOD >> 10)
#define TMEAS_BUFFER_SIZE   128//512   // ==> measuring period = TMEAS_BUFFER_SIZE*TX_COUNTER_PERIOD/VCO_FREQ (e.g. 163.84s);
                                 //caution: largest number on PIC32: 2^32. if buffer is to long the elapsed time between two points is larger than 2^32.
                                 //TMEAS_BUFFER_SIZE GOT TO BE 2^X!
//#define BUF_SUM_REF         (TX_COUNTER_PERIOD * TMEAS_BUFFER_SIZE)
#define LIM_INT 0

#define SCAL 12 //2^12
#define KI 20 //KI = 80 @ TMEAS_BUFFER_SIZE = 64, TMEAS_BUFFER_SIZE = 256 -> KI = 20
#define KP 8000 //KP = 8000 @ TMEAS_BUFFER_SIZE = 64, TMEAS_BUFFER_SIZE = 256 -> KI = 2000
#define KD 0



    
int setupDetectInterrupt(void);
BOOL updateBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx);
BOOL fillBuffer(INT32 val, INT32 *bufSum, INT32 *buf, unsigned int *bfIdx);
int sanityCheck(UINT32 edgeCount, UINT32 turns);
INT32 anotherFilter(INT32 input);
int measureFrequency(UINT32 edgeCount, INT32 *buf,
                     INT32 *pBufSum, UINT32 turns, INT32 *pError);
INT32 PID(INT32 error, UINT32 minPWMval, UINT32 maxPWMval);
int limitUnsigned(UINT32 *in, UINT32 lim);
int limitSigned(INT32 *in, INT32 plim, INT32 nlim);
INT32 secureAdd_INT32(INT32 a, INT32 b);


#ifdef	__cplusplus
}
#endif

#endif	/* MEASURING_H */

