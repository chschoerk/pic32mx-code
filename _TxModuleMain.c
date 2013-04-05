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


#define tPREAMBEL_LEN        32  //byte
#define tPREAMBEL            0x55
#define tSYNCBYTE0           0x33
#define tSYNCBYTE1           0x33
#define tSYNCBYTE2           0xA6
#define tPAYLOADLEN          32   //240 //byte

#define VCO_FREQ             12288000
#define CNT_HIST_BUFFER_SIZE 256 //size of ringbuffer that keeps track of PID controller output history
#define CNT_STOP_THRESH      20
#define CNT_STOP_SIGN_TRESH  40
#define BOUNCE_TRESH         50

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
    UINT32 turns;
    UINT32 ts = 0;
    UINT32 tsOld = 0;
    UINT32 edgeCount = 0;
    UINT32 timeStampIncrement;

    int sane;
    int ret;
    int i;
    UINT32 pwmValCurrent;
    INT32 out;
    UINT32 cntHist[CNT_HIST_BUFFER_SIZE] = { 0 };
    //UINT8 signHist[CNT_HIST_BUFFER_SIZE] = { 0 };
    INT8 devSign = 0;
    //INT16 signOff = 0;
    UINT32 cntHistBufSum = 0;
    int cntHistIdx = 0;
    //INT16 errorSignBufSum = 0;
    int x0, xm1, xm2;
    int controllerOn = 1;
    //UINT16 outSignChanges = 0;
    INT16 outOfBounceCount = 0;
    INT32 thisDeviationAbs = 0;
    //int tmpArrSize = CNT_HIST_BUFFER_SIZE;
    INT32 tmpArr1[CNT_HIST_BUFFER_SIZE] = { 0 };
    INT32 tmpArr2[CNT_HIST_BUFFER_SIZE] = { 0 };
    int tmpIdx = 0;

    counterOverflow = 0;
    counterValue = 0;
    counterValueOld = 0;
    
    /*---SYSTEM CONFIG---*/
    pbclockfreq = SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    //SYSTEMConfigWaitStatesAndPB()
    DDPCONbits.JTAGEN = 0; //disable JTAG
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    /*---PINMUXING (SWITCHING)---------------------------------------*/
    pinMux01();

    /*---SWITCHING---------------------------------------------------*/
    turnOffLED1;
    turnOffLED2;
    switchOnCounter; //enable clock division
    switch2ClockAnd(); //use buffer instead of AND
    
    /*---SETUP------------------------------------------------------*/
    setupI2S();                             //I2S (TIMESTAMP OUT)
    //setupSMBus(pbclockfreq);              //I2C (SMBus slave)
    pwmValCurrent = setupPWM(pbclockfreq);                  //PWM (VCXO CONTROL)
    setupEdgeCount();                       //VCXCO EDGE COUNTING
    turnOnLED1;
    turnOnLED2;
    setupADF();                             //ADF7023
    turnOffLED1;
    ADF_MCRRegisterReadBack(&MCRregisters); //read back the MCRRegisters
    setupDetectInterrupt();                 //PREAMBEL DETECTED IRQ

    /*---ENABLE INTERRUPTS------------------------------------------*/
    INTEnableInterrupts();

    /*---LOOP-------------------------------------------------------*/
    //stallRecover = 0;
    timeStampIncrement = (UINT32)((T1TURNS*T1PR) / (12288000/48000));
    rxDetected = FALSE;
    bOk = bOk && ADF_GoToRxState(); //ADF: Go to RX state
    turnOffLED2;
    while(1){
        
        if (rxDetected){
            toggleLED2;
            if (skippedFirst){

                i = 10000;
                while(i--); //wait

                ts = readReceivedTimestamp();
                turns = (ts - tsOld)/timeStampIncrement;
                tsOld = ts;
                edgeCount = (T1PR - counterValueOld) + ((counterOverflow-1)*T1PR) + counterValue; //only valid if counterOverflow > 0

                sane = sanityCheck(edgeCount, turns);
              
                if (sane){
                    //updateTimestamp(ts);
                    ret = measureFrequency(edgeCount, pBuf, &bufSum, turns, &fDeviation);
                    if (controllerOn == 1){
                        out = PID(-fDeviation, 0, 399999); //max: 0 - 399999´
                        if (out >= 0 && controllerOn == 1){
                            SetDCOC1PWM(out);
                        }

                        /*controller history ringbuffer*/
                        cntHistBufSum = cntHistBufSum - cntHist[cntHistIdx] + out;
                        cntHist[cntHistIdx] = out;

                        /*
                        x0 = cntHistIdx;
                        xm1 = x0 - 1;
                        xm1 &= (CNT_HIST_BUFFER_SIZE-1); //mask out top bits (wrap index)
                        xm2 = x0 - 2;
                        xm2 &= (CNT_HIST_BUFFER_SIZE-1); //mask out top bits (wrap index)
                        */
                        
                        /*check if values right and left of x0 are both lower or higher - if so we have a sign change*/
                        /*if  (  ((UINT32)(cntHist[xm1] - cntHist[xm2]) >> 31) ^ ((UINT32)(cntHist[xm1] - cntHist[x0]) >> 31) ){
                            outSignChanges = outSignChanges - signHist[cntHistIdx] + 0;
                            signHist[cntHistIdx] = 0; //no sign change
                        }else{
                            outSignChanges = outSignChanges - signHist[cntHistIdx] + 1;
                            signHist[cntHistIdx] = 1; //sign change    
                        }

                        if (outSignChanges > CNT_STOP_THRESH && ret > 0){
                            turnOnLED1;
                            out = cntHistBufSum / CNT_HIST_BUFFER_SIZE;
                            SetDCOC1PWM(out);
                            controllerOn = 0;
                        }*/

                        /*sodala, jetz aba*/
                        if (tmpArr1[cntHistIdx] > BOUNCE_TRESH){
                            outOfBounceCount--;
                        }
                        devSign = ( (fDeviation > 0) - (fDeviation < 0) );
                        thisDeviationAbs = devSign * fDeviation;
                        if (thisDeviationAbs > BOUNCE_TRESH){
                            outOfBounceCount++;
                        }
                        tmpArr1[cntHistIdx] = thisDeviationAbs;

                             
                        /*count positive and negative frequency errors*/
                        /*
                        devSign = ( (fDeviation > 0) - (fDeviation < 0) );
                        errorSignBufSum = errorSignBufSum - tmpArr1[cntHistIdx] + devSign;
                        tmpArr1[cntHistIdx] = devSign;
                        if (errorSignBufSum < 0){
                            signOff = -errorSignBufSum;
                        }else{
                            signOff = errorSignBufSum;
                        }
                        */

                        tmpArr2[cntHistIdx] = fDeviation;


                        /*check if we can stop controlling*/
                        //if (signOff < CNT_STOP_SIGN_TRESH && outSignChanges > CNT_STOP_THRESH && ret > 0 ){
                        if (outOfBounceCount < CNT_STOP_THRESH && ret > 0 ){
                            turnOnLED1;
                            //out = cntHistBufSum / CNT_HIST_BUFFER_SIZE;
                            //SetDCOC1PWM(out);
                            controllerOn = 0;
                        }

                        cntHistIdx++;
                        cntHistIdx &= (CNT_HIST_BUFFER_SIZE-1); //equals: cntHistIdx = cntHistIdx % CNT_HIST_BUFFER_SIZE if CNT_HIST_BUFFER_SIZE is 2^x

                       
                        /*DEBUG*/
                        //tmpArr1[tmpIdx] = out;
                        //tmpArr2[tmpIdx] = fDeviation;

                    }
                    /*
                    if (ret > 0){
                        turnOnLED1;
                        pwmUpdate = (float)fDeviation/(float)2013.135;
                        pwmUpdate = pwmUpdate * 2222.22;
                        pwmValCurrent -= (UINT32)(pwmUpdate/2);
                        SetDCOC1PWM(pwmValCurrent);
                        while(1){
                            ret = 1;
                        }

                        //fDeviation = anotherFilter(fDeviation); //low pass TODO  -> maybe moving average with ringbuffer 2^x
                        //out = PID(fDeviation); //limiten nicht vergessen
                        //regVal = ControlOut2PWMRegValue(out);

                        
                        
                        //TODO: set status to synced
                        //TODO: compute PWM register value
                        //TODO: set new PWM register value (SetDCOC1PWM(0x7FFF));
                    }*/
                }

            }else{ //if(skippedFirst)

                ts = readReceivedTimestamp();
                tsOld = ts;
                skippedFirst = 1;

            }
            

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
   //mPORTBToggleBits(BIT_10); //PIN 8
   mT1ClearIntFlag();
}

/*
void __ISR(_TIMER_3_VECTOR, ipl2) T3Interrupt()
{
   //mPORTBToggleBits(BIT_13);
   mT3ClearIntFlag();
}*/
