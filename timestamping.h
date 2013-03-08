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

int TS_initBuffers();
//int startDMA2_Spi1ToRxBuff(void);
int startDMA1_TxBuffToSpi1(void);


#ifdef	__cplusplus
}
#endif

#endif	/* TIMESTAMPING_H */

