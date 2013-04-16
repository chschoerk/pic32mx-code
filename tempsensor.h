/* 
 * File:   tempsensor.h
 * Author: p2773
 *
 * Created on 16. April 2013, 09:43
 */

#ifndef TEMPSENSOR_H
#define	TEMPSENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif

int setupTempSensor(void);
UINT8 readTemperature();

#ifdef	__cplusplus
}
#endif

#endif	/* TEMPSENSOR_H */

