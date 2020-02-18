/*
 * CaptCol.c
 *
 *  Created on: 21 oct. 2019
 *      Author: deloi
 */

#if defined STM32F7
#include "stm32f7xx_hal.h"
#elsif defined STM32H7
#include "stm32h7xx_hal.h"
#elsif defined STM32F4
#include "stm32f4xx_hal.h"
#elsif defined STM32L4
#include "stm32l4xx_hal.h"
#elsif defined STM32G4
#include "stm32g4xx_hal.h"
#endif

//#include "stm32h7xx_hal.h"
#include "I2CMaitre.h"
#include "Config.h"
#include "CaptCol.h"

enum {CCLNone,CCLReq,CCLLeds,CCLReqMes1,CCLReqMes2,CCLCouleur,CCLPosSv};

void CCLeds(int on)
   {
   uint8_t t[2];
   t[0]=CCLLeds;
   t[1]=on;
   I2CWriteBytes(I2CAD_COUL,t,sizeof(t));
   }

void CCCouleur(uint16_t col)
   {
   uint8_t t[2];
   t[0]=CCLCouleur;
   t[1]=col;
   I2CWriteBytes(I2CAD_COUL,t,sizeof(t));
   }

void CCPosServo(uint8_t n, uint16_t pos)
   {
   uint8_t t[3];
   uint16_t x;
   if(n>5) return;
   if(pos>1000) return;
   t[0]=CCLPosSv;
   x=pos|(n<<12);
   t[1]=x;
   t[2]=x>>8;
   I2CWriteBytes(I2CAD_COUL,t,sizeof(t));
   }

int16_t CCGetCol(unsigned capt)
   {
   HAL_StatusTypeDef err;
   uint8_t x;
   if(capt==1)
      {
      err=I2CReadBytes(I2CAD_COUL, CCLReqMes1, &x, 2);
      if(err!=HAL_OK) return -1;
      return x;
      }
   if(capt==2)
      {
      err=I2CReadBytes(I2CAD_COUL, CCLReqMes2, &x, 2);
      if(err!=HAL_OK) return -1;
      return x;
      }
   return -2;
   }

const char *StrCol(uint16_t c)
{
   static const char *sc[]={"Rouge","Bleu","Vert","Blanc","Jaune","Violet","Cyan","???"};
   if(c>sizeof(sc)/sizeof(*sc)-1) c=sizeof(sc)/sizeof(*sc)-1;
   return sc[c];
}

