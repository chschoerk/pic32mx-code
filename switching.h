/* 
 * File:   switching.h
 * Author: p2773
 *
 * Created on 19. Dezember 2012, 09:24
 */

#ifndef SWITCHING_H
#define	SWITCHING_H

#ifdef	__cplusplus
extern "C" {
#endif


#define     turnOnLED1          (mPORTCSetBits(BIT_5))
#define     turnOffLED1         (mPORTCClearBits(BIT_5))
#define     toggleLED1          (mPORTCToggleBits(BIT_5))
#define     turnOnLED2          (mPORTASetBits(BIT_9))
#define     turnOffLED2         (mPORTAClearBits(BIT_9))
#define     toggleLED2          (mPORTAToggleBits(BIT_9))

#define     switchOnCounter     (mPORTAClearBits(BIT_1))
#define     switchOffCounter    (mPORTASetBits(BIT_1))

void switch2ClockBuffer();
void switch2ClockAnd();
void switchOffAnd();
void switchOffBuffer();

#ifdef	__cplusplus
}
#endif

#endif	/* SWITCHING_H */

