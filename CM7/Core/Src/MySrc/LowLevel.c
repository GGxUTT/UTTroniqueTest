/*
 * LowLevel.c
 *
 *  Created on: May 27, 2016
 *      Author: Md
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

#include "tim.h"
#include "gpio.h"
#include "TempsMs.h"
#include "LCD_VT100.h"
#include "Config.h"
#include "LowLevel.h"

volatile uint16_t VBAT;

unsigned VitMotG(int16_t v)
    // Renvoie la valeur � mettre dans les regs P1DCx pour
    //  obtenir la vitesse v (-1000 <= v <= +1000)
{
   if((VBAT>5000)&&(VBAT<20000)) v=v*VREF/VBAT; //Compensation de la variation de la tension batterie
   if(v>1000) v=1000;
   else if(v<-1000) v=-1000;
   if(INV_MOTG) v=-v;
   v+=1000;
   v=PWMmin+v*(PWMmax-PWMmin)/2000;
   return v;
}

unsigned VitMotX(int16_t v)
    // Renvoie la valeur � mettre dans les regs P1DCx pour
    //  obtenir la vitesse v (-1000 <= v <= +1000)
{
   if((VBAT>5000)&&(VBAT<20000)) v=v*VREF/VBAT; //Compensation de la variation de la tension batterie
   if(v>1000) v=1000;
   else if(v<-1000) v=-1000;
   if(INV_MOTX) v=-v;
   v+=1000;
   v=PWMmin+v*(PWMmax-PWMmin)/2000;
   return v;
}


unsigned VitMotD(int16_t v)
    // Renvoie la valeur � mettre dans les regs P1DCx pour
    //  obtenir la vitesse v (-1000 <= v <= +1000)
{
   if((VBAT>5000)&&(VBAT<20000)) v=v*VREF/VBAT; //Compensation de la variation de la tension batterie
   if(v>1000) v=1000;
   else if(v<-1000) v=-1000;
   if(INV_MOTD) v=-v;
   v+=1000;
   v=PWMmin+v*(PWMmax-PWMmin)/2000;
   return v;
}


void SetVitMotG(int16_t v)
{
	PWMG=VitMotG(v);
}

void SetVitMotD(int16_t v)
{
	PWMD=VitMotD(v);
}

void SetVitMotX(int16_t v)
{
	PWMX=VitMotX(v);
}


void SetVitMotGD(int16_t vg, int16_t vd)
{
	unsigned g,d;
	g=VitMotG(vg);
	d=VitMotD(vd);
	PWMG=g;
	PWMD=d;
}


void Powen(int16_t st)
{
	_Powen(st);
}

void VitTurbine(float pourcent)
{
	if(pourcent<0.f)pourcent=0.f;
	if(pourcent>100.f)pourcent=100.f;
	(&htim1)->Instance->CCR3=(PWMmax*0.01f)*pourcent;
}

void PowTurbine(int16_t st)
{
	_PowTurbine(st);
}

#include "../../../Common/Src/SharedMemory.h"


int GetDetect(void)
{
	return SharedData->emergency;
}

