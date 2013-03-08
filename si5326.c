
#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>
#include <string.h>
#include "si5326.h"


#define SI53XX_CSN_DEASSERT        (mPORTBSetBits(BIT_7))
#define SI53XX_CSN_ASSERT          (mPORTBClearBits(BIT_7))



BOOL SilabSetup()
{

    return 1;
}

int SI53xx_WriteRegister(UINT8 addr, UINT8 data)
{
    SI53XX_CSN_ASSERT;
        Si53xx_XMit(CMD_SET_ADDRESS,NULL);
        Si53xx_XMit(addr,NULL);
    SI53XX_CSN_DEASSERT;

    SI53XX_CSN_ASSERT;
        Si53xx_XMit(CMD_WRITE,NULL);
        Si53xx_XMit(data,NULL);
    SI53XX_CSN_DEASSERT;

    return 0;
}

int SI53xx_ReadRegister(UINT8 addr, UINT8 *pData)
{
    SI53XX_CSN_ASSERT;
        Si53xx_XMit(CMD_SET_ADDRESS,NULL);
        Si53xx_XMit(addr,NULL);
    SI53XX_CSN_DEASSERT;

    SI53XX_CSN_ASSERT;
        Si53xx_XMit(CMD_READ,NULL);
        Si53xx_XMit(0x00,pData);
    SI53XX_CSN_DEASSERT;

    return 0;
}


void Si53xx_XMit(unsigned char ucByte,unsigned char *pData)
{
   SpiChnPutC(SPI_CHANNEL1, ucByte);

   if(pData)
      *pData = SpiChnGetC(SPI_CHANNEL1);
   else
      (void)SpiChnGetC(SPI_CHANNEL1);
}


