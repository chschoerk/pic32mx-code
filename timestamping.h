/* 
 * File:   timestamping.h
 * Author: p2773
 *
 * Created on 18. Dezember 2012, 15:25
 */

#ifndef TIMESTAMPING_H
#define	TIMESTAMPING_H

#ifdef	__cplusplus
extern "C" {
#endif


int setupI2S(void);
int initBuffers();
UINT32 readReceivedTimestamp(void);
void updateTimestamp(UINT32 timestampNew);
//void updateDMASourcePointer(void);
int fillDMABufferHalf();
int startDMA1_TxBuffToSpi1(void);
void resetSrcPtrOverruns(void);

#define TXBUFFSZ                1024     //0 < TXBUFFSZ*4 < 65636
#define TXBUFFSZ_HALF           (TXBUFFSZ/2)
#define NOTSYNCED               0
#define SYNCED                  1

#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMPING_H */

