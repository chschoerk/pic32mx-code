/* 
 * File:   configandmux.h
 * Author: p2773
 *
 * Created on 18. Dezember 2012, 11:34
 */

#ifndef CONFIGANDMUX_H
#define	CONFIGANDMUX_H

#ifdef	__cplusplus
extern "C" {
#endif

void pinMux01 (void);
void pinMux02 (void);
void SPI1_configMaster(void);
void SPI1_configSlave(void);


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIGANDMUX_H */

