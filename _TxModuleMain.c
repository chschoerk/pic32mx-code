/*
 * File:   _TxModuleMain.c
 * Author: abu
 *
 * Created on 18. Dezember 2012, 10:01
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "adf7023_mint.h"
#include "si5326.h"
#include "configandmux.h"
#include "timestamping.h"
#include "switching.h"
#include "smbus.h"
#include "measuring.h"
#include "_TxModuleMain.h"


#define tPREAMBEL_LEN       32  //byte
#define tPREAMBEL           0x55
#define tSYNCBYTE0          0x33
#define tSYNCBYTE1          0x33
#define tSYNCBYTE2          0xA6
#define tPAYLOADLEN         32   //240 //byte

#define VCO_FREQ            12288000


volatile BOOL rxDetected;
volatile unsigned int counterValue = 0;
volatile unsigned int counterValueOld = 0;
volatile unsigned int counterOverflow = 0;

//volatile int tmpIdx = 0;
//volatile int tmpSrcPtrArray[10];

volatile unsigned int stallRecover;


int main(void) {

    BOOL bOk = TRUE;
    TyMCR MCRregisters;
    unsigned char dummyDat;
    unsigned int pbclockfreq;
    int skippedFirst = 0;
    INT32 pBuf[TMEAS_BUFFER_SIZE];
    INT32 bufSum=0;
    INT32 fDeviation;
    INT32 errorVec[512];
    UINT32 turns;
    UINT32 tmpArr[100];
    UINT32 tmpTurnArr[100];
    UINT32 ts = 0;
    UINT32 tsOld = 0;
    int tmpIdx=0;
    UINT32 timeStampIncrement;
    int errIdx=0;
    int ret;
    unsigned char tsData_8[4];
    UINT32 tmp32;
    int i;

    counterOverflow = 0;
    counterValue = 0;
    counterValueOld = 0;
    
    /*---SYSTEM CONFIG---*/
    pbclockfreq = SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    //SYSTEMConfigWaitStatesAndPB()
    DDPCONbits.JTAGEN = 0; //disable JTAG
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    /*---PINMUXING (SWITCHING)---------------------------------------*/
    SwitchOffSport(); //remove with new HW
    pinMux01();
    SwitchADFSpi2Spi1(); //remove with new HW

    /*---SETUP------------------------------------------------------*/
    setupI2S();                             //I2S (TIMESTAMP OUT)
    //setupSMBus(pbclockfreq);              //I2C (SMBus slave)
    setupPWM(); //TODO 32 bit mode          //PWM (VCXO CONTROL)
    setupEdgeCount();                       //VCXCO EDGE COUNTING
    setupADF();                             //ADF7023
    ADF_MCRRegisterReadBack(&MCRregisters); //read back the MCRRegisters
    setupDetectInterrupt();                 //PREAMBEL DETECTED IRQ

    /*---ENABLE INTERRUPTS------------------------------------------*/
    INTEnableInterrupts(); 


    /*---LOOP-------------------------------------------------------*/
    //stallRecover = 0;
    timeStampIncrement = (UINT32)((T1TURNS*T1PR) / (12288000/48000));
    rxDetected = FALSE;
    bOk = bOk && ADF_GoToRxState(); //ADF: Go to RX state
    while(1){        
        if (rxDetected){
            
            if (skippedFirst){

                i = 50000;
                while(i--); //wait

                ts = updateTimestamp(); //read received data from packet ram and write to timestamp
                turns = (ts - tsOld)/timeStampIncrement;
                tsOld = ts;
                tmpArr[tmpIdx] = ts;
                tmpTurnArr[tmpIdx] = turns;
                tmpIdx++;
                if (tmpIdx == 100){
                    tmpIdx = 0;
                }
                //turns = 1; //TIMESTAMPS ARE NOT WORKING RIGHT NOW???

                if (turns == 1){
                    ret = measureFrequency(counterValue, counterValueOld, counterOverflow, pBuf, &bufSum, turns, &fDeviation);
                    if (ret > 0){
                        //fDeviation = anotherFilter(fDeviation); //or control loop (PID-Regler)
                        errorVec[errIdx] = fDeviation; //debug
                        errIdx++;
                        if(errIdx==512){
                            errIdx=0;
                        }
                        //TODO: set status to synced
                        //TODO: compute PWM register value
                        //TODO: set new PWM register value (SetDCOC1PWM(0x7FFF));
                    }
                }

            }else{ //if(skippedFirst)

                ts = updateTimestamp(); //save received timestamp
                tsOld = ts;
                tmpArr[tmpIdx] = ts;
                tmpTurnArr[tmpIdx] = -1;
                tmpIdx++;
                skippedFirst = 1;

            }
            

            /*DEBUG
            i = 50000;
            while(i--);
            bOk = ADF_MMapRead(PKT_RAM_BASE_PTR, PKT_MAX_PKT_LEN, tsData_8);
            tmp32 = tsData_8[3];
            tmp32 = (tmp32 << 8) | tsData_8[2];
            tmp32 = (tmp32 << 8) | tsData_8[1];
            tmp32 = (tmp32 << 8) | tsData_8[0];
            tmpArr[tmpIdx] = tmp32;
            if (tmpIdx > 0){
                tmpTurnArr[tmpIdx] = tmpArr[tmpIdx] - tmpArr[tmpIdx-1];
            }
            tmpIdx++;
            if (tmpIdx == 100){
                tmpIdx = 0;
            }
            */

            /*reset counters*/
            counterOverflow = 0;
            counterValueOld = counterValue;
            //rxDetected = FALSE;

            /*Clear ADF8023 Interrupt*/
            dummyDat = 0xFF;
            bOk = bOk & ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &dummyDat); //clear all interrupts in source 0 by writing 1's

            //debugging
            //RetVal = ADF_MMapRead(MCR_interrupt_source_0_Adr, 0x01, &MCRByte);
            //retBool = ADF_ReadStatus(&ADStat);

            /*update sync-status, ...*/
            //TODO

            /*Bring ADF back to RxState*/
            rxDetected = FALSE;
            bOk = bOk && ADF_GoToRxState();
                      
        }
    }

    return (EXIT_SUCCESS);
}


void __ISR(_EXTERNAL_1_VECTOR, ipl3) INT1Interrupt()
{
   //read and reset counter value TMR1
   while(T1CON & 0x0800); //check T1CON.TWIP and wait until theres no write to TMR1 in progess
   counterValue = TMR1;

   updateDMASourcePointer();
   resetSrcPtrOverruns();

   rxDetected = TRUE;
   mINT1ClearIntFlag();

}

void __ISR(_TIMER_1_VECTOR, ipl1) T1Interrupt()
{
   counterOverflow++;
   mPORTBToggleBits(BIT_2);
   mT1ClearIntFlag();
}

/*
void __ISR(_TIMER_3_VECTOR, ipl2) T3Interrupt()
{
   //mPORTBToggleBits(BIT_13);
   mT3ClearIntFlag();
}*/
