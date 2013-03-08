#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
#include <string.h>
#include "adf7023_mint.h"


/*DEFINES------------------------------------------------*/
#define DEFAULT_FREQ_DEV        25000           // 50 kHz
#define DEFAULT_DATA_RATE       96000           // 96 kbps
#define DEFAULT_CHNL_FREQ       868000000       // 868 MHz
#define FREQ_CNVRT_VAL          0.00252061538

#define ADF_CSN_DEASSERT        (mPORTCSetBits(BIT_7))
#define ADF_CSN_ASSERT          (mPORTCClearBits(BIT_7))
#define ADF_MISO_IN             (mPORTCReadBits(BIT_8))
//#define ADF_MISO_IN             (mPORTCReadBits(BIT_2)) //Alternative MISO (bitbanging)

#define bb_SET_MOSI_LOW         (mPORTCClearBits(BIT_1))
#define bb_SET_MOSI_HIGH        (mPORTCSetBits(BIT_1))
#define bb_MISO_IN              (mPORTCReadBits(BIT_2))
#define bb_SET_CLK_HIGH         (mPORTCSetBits(BIT_0))
#define bb_SET_CLK_LOW          (mPORTCClearBits(BIT_0))

//transfer settings
#define tPREAMBEL_LEN   32  //byte
#define tPREAMBEL       0x55
#define tSYNCBYTE0      0x33
#define tSYNCBYTE1      0x33
#define tSYNCBYTE2      0xA6
#define tPAYLOADLEN     PKT_MAX_PKT_LEN

/*GLOBALS------------------------------------------------*/
ADFSTA_Reg            ADF7023Status;
TyBBRAM               BBRAM;

/*
 alternative implementation of ADF_FirstConnect()
 */
BOOL ADF_Init(void)
{
    BOOL bOk = TRUE;
    ADF_FwState   FwState = ADF_GetFwState();
    bOk   = ADF_IssueCommandNW(CMD_HW_RESET);
   /*datasheet pg 35: Init after issuing CMD_HW_RESET:
    * The CMD_HW_RESET command performs a full power-down of all hardware, and the
    * device enters the PHY_SLEEP state. To complete the hardware reset, the host
    * processor should complete the following procedure:
    *  1. Wait for 1 ms.
    *  2. Bring the CS pin of the SPI low and wait until the MISO output goes high.
    *     The ADF7023 registers a POR and enters the PHY_OFF state.
    *  3. Poll status word and wait for the CMD_READY bit to go high.
    *  4. Configure the part by writing to all 64 of the BBRAM registers.
    *  5. Issue the CMD_CONFIG_DEV command so that the radio settings are updated using the BBRAM values.
    *
    * The ADF7023 is now configured in the PHY_OFF state.
   */
   int j = 500000;
   while(j--); //wait...

   ADF_CSN_DEASSERT;
   ADF_CSN_ASSERT;
   bOk = bOk && ADF_waitForMISOToGoHigh();
   //ADF_CSN_DEASSERT; // bring CS high again?

   bOk = bOk && ADF_WaitCmdLdr();




   FwState = ADF_GetFwState();

   // handle all possible states that the ADF7023 could be in and brings it back to PHY_ON state
   while ((FwState != FW_OFF) && bOk)
       {
       switch (FwState)
           {
           case FW_BUSY:
               break;
           case FW_TX:
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_ON);
               bOk   = bOk && ADF_WaitFWState   (FW_ON);
               break;
           case FW_RX:
               // Transition modes
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_ON);
               bOk   = bOk && ADF_WaitFWState   (FW_ON);
               break;
           default:
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_OFF);
               bOk   = bOk && ADF_WaitFWState   (FW_OFF);
               break;
           }
       FwState = ADF_GetFwState();
       }




   ADF_BBRAMDefault(&BBRAM); // Generate our default radio setup

   bOk = bOk && ADF_MMapWrite(BBRam_MMemmap_Start, sizeof(TyBBRAM), (unsigned char *)&BBRAM); // Commit the local BBRAM to the Radio

   bOk = bOk && ADF_IssueCommand(CMD_CONFIG_DEV); // Update MCRs


   //write to MCRs (e.g. GPIO configure)...
   //unsigned char gpioMode = gpio_configure_sport_mode_4;
   unsigned char gpioMode = gpio_configure_default;
   bOk = bOk && ADF_MMapWrite(MCR_gpio_configure_Adr, 0x1, (unsigned char *) &gpioMode);
   //Interrupt on GP4 (if data mode = 1 -> irq on preambel detect, if data mode = 2 -< irq on syncword detect)

   bOk = bOk && ADF_GoToOnState();  // Part in the PHY_ON mode
   //go to PHY_ON mode, CMD_CONFIG_DEV can be issue in PHY_OFF or PHY_ON

   return bOk;
}

BOOL ADF_waitForMISOToGoHigh()
{
    int      iTmp;
   BOOL  RetVal = TRUE;
   int      i = 0x0;

   // Wait for MISO line to go high
   // Wait for ADF to wake up (only necessary for first byte in packet)
   iTmp = 0; // clear before following line
   // monitor MISO line (P1.5)
   while (0 == iTmp && i < 1000) { //check whether bit 5 is high
       iTmp = ADF_MISO_IN;
       i++;
   }
   if (1000 == i)
      RetVal = FALSE;
   else
      RetVal = TRUE;

   ADF_CSN_DEASSERT;   // De-assert SPI chip select

   return RetVal;
}


BOOL ADF_MCRRegisterReadBack(TyMCR *pMCRin)
{
    BOOL retVal;
    retVal = ADF_MMapRead(MCR_MMemmap_Start, sizeof(TyMCR), (unsigned char *)pMCRin);
    return retVal;
}


/*************************************************************************/
/* void ADF_FirstConnect(void)                                           */
/* Parameters:                                                           */
/*   Connecting to the ADF7023 for the first time:                       */
/*   after POR or after bottom die reset.                                */
/*   this function will be called once. BBRAM is initialised here        */
/*************************************************************************/
BOOL        ADF_FirstConnect       (void)
{
   BOOL        bOk = TRUE;
   ADF_FwState   FwState = ADF_GetFwState();

   bOk   = bOk && ADF_IssueCommandNW    (CMD_HW_RESET);
   /*datasheet pg 35: Init after issuing CMD_HW_RESET:
    * The CMD_HW_RESET command performs a full power-down of all hardware, and the
    * device enters the PHY_SLEEP state. To complete the hardware reset, the host
    * processor should complete the following procedure:
    *  1. Wait for 1 ms.
    *  2. Bring the CS pin of the SPI low and wait until the MISO output goes high.
    *     The ADF7023 registers a POR and enters the PHY_OFF state.
    *  3. Poll status word and wait for the CMD_READY bit to go high.
    *  4. Configure the part by writing to all 64 of the BBRAM registers.
    *  5. Issue the CMD_CONFIG_DEV command so that the radio settings are updated using the BBRAM values.
    *
    * The ADF7023 is now configured in the PHY_OFF state.
   */

   bOk   = bOk && ADF_SyncComms(); // issue CMD_SYNC (probably not needed, but CS is pulled low here), wait for CMD_READY bit to go high

   FwState = ADF_GetFwState();

   // handle all possible states that the ADF7023 could be in and brings it back to PHY_ON state
   while ((FwState != FW_OFF) && bOk)
       {
       switch (FwState)
           {
           case FW_BUSY:
               break;
           case FW_TX:
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_ON);
               bOk   = bOk && ADF_WaitFWState   (FW_ON);
               break;
           case FW_RX:
               // Transition modes
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_ON);
               bOk   = bOk && ADF_WaitFWState   (FW_ON);
               break;
           default:
               bOk   = bOk && ADF_IssueCommandNW(CMD_PHY_OFF);
               bOk   = bOk && ADF_WaitFWState   (FW_OFF);
               break;
           }
       FwState = ADF_GetFwState();
       }

   // Generate our default radio setup
   ADF_BBRAMDefault(&BBRAM);

   // Update the radio
   bOk = bOk && ADF_ConfigureRadio(&BBRAM);
   /*- write to BBRAM
    *- issue CMD_PHY_ON
    *- issue CMD_CONFIG_DEV to update radio settings
    */

   // Part in the PHY_ON mode
   bOk = bOk && ADF_GoToOnState();

   return bOk;
}

/*************************************************************************/
/* void ADF_SyncComms(void)                                            */
/* Parameters:                                                           */
/*************************************************************************/
BOOL        ADF_SyncComms       (void)
{
   BOOL        bOk = TRUE;
   bOk   = bOk && ADF_IssueCommandNW(CMD_SYNC);    // will synchronise processors, recommended seq at power up
   bOk   = bOk && ADF_WaitCmdLdr  ();
   return bOk;
}



/******************************************************************************/
/* Function    : ADF_XMitByte                                                 */
/* Description : Transmit a byte over SPI and wait for response               */
/*               Update status or return data                                 */
/******************************************************************************/
void ADF_XMit(unsigned char ucByte,unsigned char *pData)
{
   SpiChnPutC(SPI_CHANNEL1, ucByte);

   /*SEND_SPI(ucByte);  // Send byte
   WAIT_SPI_RX;  // wait for data received status bit*/
   if(pData)
      *pData = SpiChnGetC(SPI_CHANNEL1);
   else
      (void)SpiChnGetC(SPI_CHANNEL1);
}


void ADF_XMit_softwareSPI(unsigned char ucByte,unsigned char *pData) //_bitbang
{
    int tIn;
    unsigned char tInByte = 0;
    int m = 0;

    for (m=0; m<8;m++){

        tInByte = tInByte << 1;

        if (ucByte & 0x80)
            bb_SET_MOSI_HIGH;
        else
            bb_SET_MOSI_LOW;

        //wait for a while?
        bb_SET_CLK_HIGH; //ADF samples data
        if (bb_MISO_IN) //read data
            tInByte = tInByte | 1;


        ucByte = ucByte << 1;
        bb_SET_CLK_LOW; //ADF shifts out next data bit

    }

    if(pData)
        *pData = tInByte;

}

/******************************************************************************/
/* Function    : ADF_IssueCommandNW                                           */
/* Description : Issue specified command                                      */
/******************************************************************************/
BOOL  ADF_IssueCommandNW(unsigned char Cmd)
{
   int      iTmp;
   BOOL  RetVal = TRUE;
   int      i = 0x0;

   // De-assert SPI chip select
   ADF_CSN_DEASSERT;
   // Assert SPI chip select
   ADF_CSN_ASSERT;

   // Wait for MISO line to go high
   // Wait for ADF to wake up (only necessary for first byte in packet)
   iTmp = 0; // clear before following line
   // monitor MISO line (P1.5)
   while (0 == iTmp && i < 1000) { //check whether bit 5 is high
       iTmp = ADF_MISO_IN; // bits 7:0 are current POrt1 data input
       i++;
   }
   if (1000 == i)
      RetVal = FALSE;
   else
      {
      ADF_XMit(Cmd,NULL); // Send Command
      }
   ADF_CSN_DEASSERT;   // De-assert SPI chip select
   return RetVal;
}


/*!
    \fn      BOOL  ADF_ReadStatus(ADFSTA_Reg *pStatus)
    \brief   Read the SPI Status Byte.

             This should be done before any other functions are called.

             A wire reset, followed by the switch sequence from JTAG to SW
             , followed again by a wire reset is performed. This is suitable
             to put either a SW-DP or SWJ-DP into IDLE mode in Serial Wire
             mode.

             The device that is present is detected and the Watchdog is
             turned off if it is not already locked on.


    \param   pStatus      Pointer to storage.

    \return  BOOL TRUE, if function successful, else FALSE
*/
BOOL  ADF_ReadStatus(ADFSTA_Reg *pStatus)
{
   int      iTmp;
   BOOL  RetVal = TRUE;
   int      i = 0x0;

   ADF_CSN_DEASSERT; // De-assert SPI chip select
   ADF_CSN_ASSERT;   // Assert SPI chip select

   // Wait for MISO line to go high after assertion of CSN
   iTmp = 0;

   // monitor MISO line (P1.5)
   while (0 == iTmp && i < 1000)
       {
       iTmp = ADF_MISO_IN;
       i++;
       }

   if (1000 == i)
      RetVal = FALSE; // ADF7023 didn't respond
   else
      {
      // Send Command
      ADF_XMit(SPI_NOP,NULL);
      ADF_XMit(SPI_NOP,(unsigned char *)pStatus);
      }
   ADF_CSN_DEASSERT; // De-assert SPI chip select
   return RetVal;
}

/******************************************************************************/
/* Function    : ADF_WaitCmdLdr                                               */
/* Description : Wait for the command loader to be empty                      */
/******************************************************************************/
BOOL  ADF_WaitCmdLdr(void)
{
   BOOL  RetVal = TRUE;
   do
      {
      RetVal   = ADF_ReadStatus(&ADF7023Status);
      }
   while(RetVal && ((ADF7023Status.Bits.cmd_loader_empty == 0x0)));
   return RetVal;
}

/******************************************************************************/
/* Function    : ADF_GetFwState                                                */
/* Description :                          */
/******************************************************************************/
ADF_FwState ADF_GetFwState(void)
{
  (void)ADF_ReadStatus(&ADF7023Status);
  return ADF7023Status.Bits.fw_state;
}


/******************************************************************************/
/* Function    : ADF_WaitFWState                                                */
/* Description : Wait until firmware state is reached                          */
/******************************************************************************/
BOOL  ADF_WaitFWState(ADF_FwState FWState)
{
   BOOL  RetVal = TRUE;
   do
      {
      RetVal   = ADF_ReadStatus(&ADF7023Status);
      }
   while(RetVal && (ADF7023Status.Bits.fw_state != FWState));
   return RetVal;
}


/*************************************************************************/
/* void ADF_ConfigureRadio(void)                                         */
/* Parameters:                                                           */
/*************************************************************************/
BOOL        ADF_ConfigureRadio       (TyBBRAM *pBBRAM)
{
   BOOL        bOk = TRUE;

   // Use current if not otherwise specified
   if (pBBRAM == NULL)
      pBBRAM = &BBRAM;

   // Commit the local BBRAM to the Radio
   bOk = bOk && ADF_MMapWrite(BBRam_MMemmap_Start, sizeof(TyBBRAM), (unsigned char *)pBBRAM);

   // Part in the PHY_ON mode
   bOk = bOk && ADF_GoToOnState();

   // Update MCRs
   bOk = bOk && ADF_IssueCommand(CMD_CONFIG_DEV);

   return bOk;
}

/******************************************************************************/
/* Function    : ADF_IssueCommand                                             */
/* Description : Issue specified command                                      */
/******************************************************************************/
BOOL  ADF_IssueCommand(unsigned char Cmd)
{
   BOOL  RetVal = TRUE;
   RetVal = RetVal && ADF_WaitCmdLdr();
   if(RetVal)
      RetVal = ADF_IssueCommandNW(Cmd);
   return RetVal;
}



/******************************************************************************/
/* Function    : MMapReadByte                                                 */
/* Description : Read Byte from specified memory map address                  */
/******************************************************************************/
unsigned char ADF_MMapRead    (unsigned long ulAdr, unsigned long ulLen, unsigned char *pData)
{
   int iTmp,RetVal = TRUE;
   int i = 0x0;
   if(RetVal)
      {
      ADFSTA_MemCmd Cmd;
      // Assemble the command
      Cmd.Bits.Cmd  = SPI_MEM_RD >> 3;
      Cmd.Bits.AdrH = (ulAdr & 0x700) >> 8;
      Cmd.Bits.AdrL = ulAdr & 0xFF;
      // De-assert SPI chip select
      ADF_CSN_DEASSERT;
      // Assert SPI chip select
      ADF_CSN_ASSERT;

      // Wait for MISO line to go high
      // Wait for ADF to wake up (only necessary for first byte in packet)
      iTmp = 0; // clear before following line
      // monitor MISO line
      while (0 == iTmp && i < 1000) {
          iTmp = ADF_MISO_IN; // bits 7:0 are current POrt1 data input
          i++;
      }
      if (1000 == i)
         RetVal = FALSE; //
      else
         {
         // Send first byte (SPI_MEMR_RD + Bytes)
         ADF_XMit(Cmd.ucBytes[0x0],NULL);
         // Send Second byte remainder of addrress
         ADF_XMit(Cmd.ucBytes[0x1],NULL);
         // Send NOP
         ADF_XMit(SPI_NOP,NULL);
         // Send NOP Data available now
         while(ulLen--)
            ADF_XMit(SPI_NOP,pData++);
         }
      ADF_CSN_DEASSERT; // De-assert SPI chip select
      }
   return RetVal;
}


/******************************************************************************/
/* Function    : MMapWrite                                                    */
/* Description : Write Byte to specified memory map address                   */
/******************************************************************************/
unsigned char ADF_MMapWrite(unsigned long ulAdr,
                            unsigned long ulLen,
                            unsigned char *pData)
{
   int iTmp,RetVal = TRUE;
   int i = 0x0;

   if(RetVal)
      {
      ADFSTA_MemCmd Cmd;

      // Assemble the command
      Cmd.Bits.Cmd  = SPI_MEM_WR >> 3;
      Cmd.Bits.AdrH = (ulAdr & 0x700) >> 8;
      Cmd.Bits.AdrL = ulAdr & 0xFF;

      // De-assert SPI chip select
      ADF_CSN_DEASSERT;
      // Assert SPI chip select
      ADF_CSN_ASSERT;

      // Wait for MISO line to go high
      // Wait for ADF to wake up (only necessary for first byte in packet)
      //
      iTmp = 0; // clear before following line
      // monitor MISO line (P1.5)
      while (0 == iTmp && i < 1000) { //check whether bit 5 is high
          iTmp = ADF_MISO_IN; // bits 7:0 are current POrt1 data input
          i++;
      }
      if (1000 == i)
         RetVal = FALSE; //
      else
         {
         // Send first byte (SPI_MEMR_WR + Bytes)
         ADF_XMit(Cmd.ucBytes[0x0],NULL);
         // Send Second byte remainder of addrress
         ADF_XMit(Cmd.ucBytes[0x1],NULL);
         // Send Data
         //
         while(ulLen--)
            ADF_XMit(*(pData++),NULL);
         }
      // De-assert SPI chip select
      ADF_CSN_DEASSERT;
      }

   return RetVal;
}

/*************************************************************************/
/* void ADF_GoToOnState(void)                    */
/* Parameters:          */
/*************************************************************************/
BOOL ADF_GoToOnState(void)
{
   BOOL        bOk = TRUE;
   bOk   = bOk && ADF_IssueCommand(CMD_PHY_ON); // Enter PHY_ON mode
   if (bOk)
        bOk   = bOk && ADF_WaitFWState     (FW_ON);
   return bOk;
}

/*************************************************************************/
/* void ADF_GoToRxState(void)                    */
/* Parameters:          */
/*************************************************************************/
BOOL ADF_GoToRxState(void)
{
   BOOL        bOk = TRUE;
   bOk   = bOk && ADF_IssueCommand(CMD_PHY_RX); // Enter PHY_ON mode
   /*TEMP: don't wait for status*/
   //if (bOk)
   //     bOk   = bOk && ADF_WaitFWState     (FW_RX);
   //return bOk;
}

/*************************************************************************/
/* void ADF_GoToRxState(void)                    */
/* Parameters:          */
/*************************************************************************/
BOOL ADF_GoToTxState(void)
{
   BOOL        bOk = TRUE;
   bOk   = bOk && ADF_IssueCommand(CMD_PHY_TX); // Enter PHY_TX mode
   if (bOk)
        bOk   = bOk && ADF_WaitFWState     (FW_TX);
   return bOk;
}

/*************************************************************************/
/* void ADF_SetChannelFreq(TyBBRAM *pBBRAM,unsigned long ulChannelFreq)  */
/*************************************************************************/
void ADF_SetChannelFreq(TyBBRAM *pBBRAM,unsigned long ulChannelFreq)
{
   // Use current if not otherwise specified
   if (pBBRAM == NULL)
      pBBRAM = &BBRAM;

   // The RF channel frequency bits [7:0] in Hz is set according to:
   // Frequency(Hz) = FPFD x channel_Freq[23:0] /2^16
   // where FPFD is the PFD frequency and is equal to 26MHz
   //
   ulChannelFreq = (unsigned long)(ulChannelFreq * FREQ_CNVRT_VAL);
   pBBRAM->channel_freq_0_r                     = (ulChannelFreq >> 0) & 0xFF;
   pBBRAM->channel_freq_1_r                     = (ulChannelFreq >> 8) & 0xFF;
   pBBRAM->channel_freq_2_r                     = (ulChannelFreq >> 16)& 0xFF;
}

/*************************************************************************/
/* void ADF_SetFreqDev          (TyBBRAM *pBBRAM,unsigned long ulFreqDev)*/
/*************************************************************************/
void ADF_SetFreqDev          (TyBBRAM *pBBRAM,unsigned long ulFreqDev)
{
   // Use current if not otherwise specified
   if (pBBRAM == NULL)
      pBBRAM = &BBRAM;

   // The binary level FSK frequency deviation in Hz (defined as
   // frequency difference between carrier frequency and 1/0 tones) is
   // set according to:
   // Frequency Deviation (Hz) = Freq_Deviation[11:0] x100

   // Frequency Deviation
   pBBRAM->radio_cfg_2_r = (unsigned char)((ulFreqDev / 100) & 0xFF);
   pBBRAM->radio_cfg_1_r &= 0x0F; // Upper nibble of radio_cfg_1_r is used for Frequency Deviation[11:8]
   pBBRAM->radio_cfg_1_r |= (unsigned char)(((ulFreqDev / 100) & 0xF00) >> 4);;
}
/*************************************************************************/
/* void ADF_SetDataRate         (TyBBRAM *pBBRAM,unsigned long ulDataRate)*/
/*************************************************************************/
void ADF_SetDataRate         (TyBBRAM *pBBRAM,unsigned long ulDataRate)
{
   // Use current if not otherwise specified
   if (pBBRAM == NULL)
      pBBRAM = &BBRAM;

   // The datarate in bps is set according to:
   // DataRate (bps) = data_rate[11:0] x 100
   pBBRAM->radio_cfg_0_r = (unsigned char)((ulDataRate / 100) & 0xFF);
   pBBRAM->radio_cfg_1_r &= 0xF0; // Bottom nibble of radio_cfg_1_r is used for data_rate[11:8]
   pBBRAM->radio_cfg_1_r |= (unsigned char)(((ulDataRate / 100) & 0xF00) >> 8);;
}


/*************************************************************************/
/* void ADF_BBRAMDefault(void)                                           */
/* Setup the default configuration parameters in all modes                */
/* Parameters:                                                           */
/*    None.                                                            */
/*************************************************************************/
void ADF_BBRAMDefault(TyBBRAM *pBBRAM)
{
   int      iOffset;
   // Initialise
   memset(pBBRAM,0x0,sizeof(TyBBRAM));

   // Configure which events will be flagged to the Cortex via interrupt
   pBBRAM->interrupt_mask_0_r                   = interrupt_mask_0_interrupt_tx_eof          | // Packet transmitted
                                                  interrupt_mask_0_interrupt_sync_detect     |
                                                  //interrupt_mask_0_interrupt_crc_correct     | // Packet received
                                                  interrupt_mask_0_interrupt_aes_done;

   pBBRAM->interrupt_mask_1_r                   = 0x0;

   // These can be initialised to zero
   // Internal 16-bit count of the number of wake ups (wuc timeouts) the device has gone through
   pBBRAM->number_of_wakeups_0_r                = 0x0;
   pBBRAM->number_of_wakeups_1_r                = 0x0;

   // This is the threshold for the number of wakeups
   // (wuc timeouts). It is a 16-bit count threshold that is compared
   // against the number_of_wakeups. When this threshold is exceeded
   // the device wakes up into the state PHY_OFF and optionally
   // generates interrupt_num_wakeups.
   pBBRAM->number_of_wakeups_irq_threshold_0_r  = 0xFF;
   pBBRAM->number_of_wakeups_irq_threshold_1_r  = 0xFF;

   // When the WUC is used and SWM is enabled), then the radio
   // powers up and enables the receiver on the channel defined in the
   // BBRAM and listens for this period of time. If no preamble pattern is
   // detected in this period, the device goes back to sleep.
   pBBRAM->rx_dwell_time_r                      = 0x0;

   // Units of time used to define the rx_dwell_time time period.
   pBBRAM->parmtime_divider_r                   = 0x33;  // 995.7Hz

   // This sets the RSSI threshold when in Smart Wake Mode with RSSI
   // detection enabled.
   // Threshold (dBm) = listen_rssi_thresh - 119
   pBBRAM->swm_rssi_thresh_r                    = 0x31; // -58dBm

   // Set the channel frequency
   ADF_SetChannelFreq(pBBRAM,DEFAULT_CHNL_FREQ);

   // The datarate
   ADF_SetDataRate(pBBRAM,DEFAULT_DATA_RATE);

   // Frequency Deviation
   ADF_SetFreqDev(pBBRAM,DEFAULT_FREQ_DEV);

   // RX Setting
   // Discriminator bandwidth for 96K data rate, 50k freq_dev, 200k IF freq, 10k freq error, AFC on (datasheet page 78)
   pBBRAM->radio_cfg_3_r = 0x31; // discriminator_bw

  // Post-demodulator bandwidth
   pBBRAM->radio_cfg_4_r = (unsigned char)((0.00075*0.73*DEFAULT_DATA_RATE)-5); // post_demod_bw

   // Reserved
   pBBRAM->radio_cfg_5_r = 0x00; // reserved

   // radio_cfg_6
   pBBRAM->radio_cfg_6_r = 0x02; // synth_lut_config_0 | discrim_phase[1:0]

   // agc_mode, Synth_Lut_control, Synth_LUT_config_1
   /*
   pBBRAM->radio_cfg_7_r =  SET_BITS(0,
                                     radio_cfg_7_synth_lut_config_1_numbits,
                                     radio_cfg_7_synth_lut_config_1_offset,
                                     0)                                                |
                                     radio_cfg_7_synth_lut_control_predef_rx_predef_tx |
                                     radio_cfg_7_agc_lock_mode_lock_after_preamble     ;

   */
   pBBRAM->radio_cfg_7_r = 0x00                                              | //synth_lut_config_1 = 0
                           radio_cfg_7_synth_lut_control_predef_rx_predef_tx |
                           radio_cfg_7_agc_lock_mode_lock_after_preamble     ;
   // PA settings
   pBBRAM->radio_cfg_8_r = radio_cfg_8_pa_single_diff_sel_single_ended |
                           radio_cfg_8_pa_power_setting_63             |
                           radio_cfg_8_pa_ramp_16;

   // Modulation/Demodulation Scheme
   // FSK/FSK/150 Hz receiver filter bandwidth
   pBBRAM->radio_cfg_9_r = radio_cfg_9_demod_scheme_FSK        |
                           radio_cfg_9_mod_scheme_2_level_FSK  |
                           radio_cfg_9_ifbw_100kHz;
   // AFC
   pBBRAM->radio_cfg_10_r = radio_cfg_10_afc_polarity_fixed_value            |
                            radio_cfg_10_afc_scheme_fixed_value              |
                            radio_cfg_10_afc_lock_mode_lock_after_preamble;

   // Sets the AFC PI controller proportional gain.
   // Sets the AFC PI controller integral gain.
   pBBRAM->radio_cfg_11_r   = radio_cfg_11_afc_kp_2_power_3 | radio_cfg_11_afc_ki_2_power_7; //recommended values

   // image_reject_cal_phase
   pBBRAM->image_reject_cal_phase_r   = 0x0; //or maybe 0x16
   // image_reject_cal_amplitude
   pBBRAM->image_reject_cal_amplitude_r   = 0x0; //or maybe 0x07

   pBBRAM->mode_control_r = mode_control_swm_en_disabled                        |
                            mode_control_bb_cal_enabled                         |
                            mode_control_swm_rssi_qual_disabled                 |
                            mode_control_tx_auto_turnaround_disabled            |
                            mode_control_rx_auto_turnaround_disabled            |
                            mode_control_custom_trx_synth_lock_time_en_disabled |
                            mode_control_ext_lna_en_disabled                    |
                            mode_control_ext_pa_en_disabled                     ;

   // Number of preamble bit errors in 24 bit window (RX)
   // value of <4 bit errors is recommended to prevent false preamble detections
   pBBRAM->preamble_match_r = preamble_match_3_in_24_win; //zero errors allowed, maybe change this

   // Symbol Paramters
   pBBRAM->symbol_mode_r = symbol_mode_symbol_length_8_bit          |
                           //symbol_mode_data_whitening_disabled      |
                           symbol_mode_data_whitening_enabled       |
                           symbol_mode_eight_ten_enc_disabled       |
                           symbol_mode_prog_crc_en_disabled         | // Default CRC selected (x16 + x15 + x2 + 1)
                           symbol_mode_manchester_enc_disabled ;

   // Length of (TX) preamble in bytes. Example a value of decimal 3
   // results in a preamble of 24 bits.
   pBBRAM->preamble_len_r = tPREAMBEL_LEN; //in byte

   // crc_poly[15:0], which sets the CRC polynomial (using default in symbol_mode).
   pBBRAM->crc_poly_0_r  = 0x00;
   pBBRAM->crc_poly_1_r  = 0x00;

   // Sets the sync word error tolerance in bits.
   // Sets the sync word length in bits. 24 bits is the maximum.
   // Note that the sync word matching length can be any value up to 24 bits
   // , but the transmitted sync word pattern is a multiple of 8 bits.
   // Hence, for non-byte-length sync words, the transmitted sync pattern
   // should be filled out with the preamble pattern.
   pBBRAM->sync_control_r = sync_control_sync_error_tol_2_errors_allowed |
                            sync_control_sync_word_length_24;
   // The sync word pattern is transmitted most significant bit first
   // starting with sync_byte[7:0].
   // For non-byte length sync words the reminder of the least
   // significant byte should be stuffed with preamble.
   // If sync_word_length length is >16 bits then sync_byte_0,
   // sync_byte_1 and sync_byte_2 are all transmitted for a total of 24 bits.
   // If sync_word_length is between 8 and 15 then sync_byte_1 and sync_byte_2
   // are transmitted.
   // If sync_word_length is between 1 and 7 then sync_byte_2 is
   // transmitted for a total of 8 bits.
   // If the sync word length is 0 then no sync bytes are transmitted.
   pBBRAM->sync_byte_0_r  = tSYNCBYTE0;
   pBBRAM->sync_byte_1_r  = tSYNCBYTE1;
   pBBRAM->sync_byte_2_r  = tSYNCBYTE2;


   // Address in Packet RAM of transmit packet. This address
   // indicates to the comms processor the location of the
   // first byte of the transmit packet
   pBBRAM->tx_base_adr_r            = PKT_RAM_BASE_PTR;

   // Address in Packet RAM of receive packet. The communications
   // processor will write any qualified received packet to Packet RAM,
   // starting at this memory location.
   pBBRAM->rx_base_adr_r            = PKT_RAM_BASE_PTR;

   // Various packet options
   pBBRAM->packet_length_control_r  = packet_length_control_data_byte_lsb         | // LSB
                                      //packet_length_control_packet_len_variable   | // Variable packet length
                                      packet_length_control_packet_len_fixed      | // Fixed packet length
                                      //packet_length_control_crc_en_yes            | // CRC Enabled
                                      packet_length_control_crc_en_no             | // CRC Disabled
                                      packet_length_control_data_mode_packet      | // No sport
                                      //packet_length_control_data_mode_sport_sync  | // SPORT mode
                                      packet_length_control_length_offset_minus0;   // For variable length packets where the first byte (length) needs to be adjusted

   pBBRAM->packet_length_max_r      = tPAYLOADLEN;

   // Set to 0x00
   pBBRAM->static_reg_fix_r         = 0x0;

   // Location of first byte of address information in packet RAM (relative to rx_base)
   pBBRAM->address_match_offset_r   = ADR_MATCH_OFFSET;

   pBBRAM->address_length_r = 0x00; //no address filtering

   iOffset = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   pBBRAM->address_filtering_r[iOffset++]     = 0x0;
   
   pBBRAM->rssi_wait_time_r = 0xA7; //recommedation in datasheet, other values can be used
   pBBRAM->testmodes_r = 0x00; //see testmodes
   pBBRAM->transition_clock_div_r = 0x00;

   pBBRAM->rx_synth_lock_time_r       = 0x0;
   pBBRAM->rx_synth_lock_time_r       = 0x0;
}



