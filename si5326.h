/*
 * File:   si5326.h
 * Author: p2773
 *
 * Created on 17. Dezember 2012, 12:31
 */

#ifndef SI5326_H
#define	SI5326_H

#ifdef	__cplusplus
extern "C" {
#endif


/*Commands*/
#define CMD_SET_ADDRESS         0x00
#define CMD_WRITE               0x40
#define CMD_WRITE_INC_ADRESS    0x60
#define CMD_READ                0x80
#define CMD_READ_INC_ADDRESS    0xa0

/*Registers*/
#define REG0    0
#define REG1    1
#define REG2    2
#define REG3    3
#define REG4    4
#define REG5    5
#define REG6    6
#define REG7    7
#define REG8    8
#define REG9    9
#define REG10   10
#define REG11   11
#define REG16   16
#define REG17   17
#define REG18   18
#define REG19   19
#define REG20   20
#define REG21   21
#define REG22   22
#define REG23   23
#define REG24   24
#define REG25   25
#define REG31   31
#define REG32   32
#define REG33   33
#define REG34   34
#define REG35   35
#define REG36   36
#define REG40   40
#define REG41   41
#define REG42   42
#define REG43   43
#define REG44   44
#define REG45   45
#define REG46   46
#define REG47   47
#define REG48   48
#define REG55   55
#define REG128  128
#define REG129  129
#define REG130  130
#define REG131  131
#define REG132  132
#define REG134  134
#define REG135  135
#define REG136  136
#define REG138  138
#define REG139  139
#define REG142  142
#define REG143  143

/*Register Bits*/
#define free_run_dis            (0 << 6)
#define free_run_en             (1 << 6)
#define ckout_always_on_squelch (0 << 5)
#define ckout_always_on_provide (1 << 5)
#define bypass_reg_normalop     (0 << 1)
#define bypass_reg_bypassmode   (1 << 1)

#define ck_prior2_ckin1issecond (0 << 2)
#define ck_prior2_ckin2issecond (1 << 2)
#define ck_prior1_ckin1isfirst  (0 << 2)
#define ck_prior1_ckin2isfirst  (1 << 2)



/*Function Prototypes*/
BOOL    SilabSetup();
void    Si53xx_XMit(unsigned char ucByte,unsigned char *pData);
int     SI53xx_WriteRegister(UINT8 addr, UINT8 data);
int     SI53xx_ReadRegister(UINT8 addr, UINT8 *pData);




#ifdef	__cplusplus
}
#endif

#endif	/* SI5326_H */

