/*
 * RT-Main.c
 *
 *  Created on: 19 août 2017
 *      Author: Md
 */

#include "main.h"

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


//#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
//#include "dma.h"
#include "i2c.h"
//#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "TempsMs.h"
#include "LCD_VT100.h"
#include "Config.h"
#include "LowLevel.h"
#include "Tests.h"
#include "Odometrie.h"
#include "Sequenceur.h"
#include "spidma.h"
#include "CommandesServos.h"
#include "I2CUSCommun.h"
#include "Asserv.h"
#include "I2CMaitre.h"
#include "ServoDefs.h"
#include "Match.h"
#include "Menu.h"
#include "MotX.h"
#include "CaptCol.h"


#include "RT-Main.h"

TaskHandle_t hMain;

#define USE_TIRETTE 1

void Coucou(int n, uint16_t d)
   {
   int i;
   for(i=0; i<n; i++)
      {
	  _LEDJ(0);
      _LEDR(1);
      DelaiMs(d);
      _LEDR(0);
      _LEDV(1);
      DelaiMs(d);
      _LEDV(0);
      _LEDJ(1);
      DelaiMs(d);
      }
   _LEDJ(0);
   }



void DbgServoMoveW(uint16_t id, uint16_t pos)
{
   ServoMoveW(id,pos);
   lcdprintf("MoveW %s %u : %s\n",GetSvName(id),pos,GetSvErrorMsg());
}


#if 0
static volatile uint16_t TLR, TLV, TLO, TLB;
void GestLeds(void)
   {
   if(TLR)
      {
      TLR--;
      if(!TLR) _LED_ROUGE(0);
      }
   if(TLV)
      {
      TLV--;
      if(!TLV) _LED_VERTE(0);
      }
   if(TLO)
      {
      TLO--;
      if(!TLO) _LED_ORANGE(0);
      }
   if(TLB)
      {
      TLB--;
      if(!TLB) _LED_BLEUE(0);
      }

   }


void _LED_ROUGE_ON(uint16_t ms)
   {
   if(!ms) return;
   _LED_ROUGE(1);
   TLR=ms;
   }

void _LED_VERTE_ON(uint16_t ms)
   {
   if(!ms) return;
   _LED_VERTE(1);
   TLV=ms;
   }

void _LED_ORANGE_ON(uint16_t ms)
   {
   if(!ms) return;
   _LED_ORANGE(1);
   TLO=ms;
   }

void _LED_BLEUE_ON(uint16_t ms)
   {
   if(!ms) return;
   _LED_BLEUE(1);
   TLB=ms;
   }
#endif


#if 0
void USStop(void)
   {
   I2CWriteByte(I2C_AD_US,CUSStop);
   }

void USAV(void)
   {
   I2CWriteByte(I2C_AD_US,CUSAV);
   }

void USAR(void)
   {
   I2CWriteByte(I2C_AD_US,CUSAR);
   }

void USAVAR(void)
   {
   I2CWriteByte(I2C_AD_US,CUSAVAR);
   }

void USAlt(uint16_t mode)
   {
   uint8_t t[2];
   t[0]=CUSAlt;
   t[1]=mode&3;
   I2CWriteBytes(I2C_AD_US,t,sizeof(t));
   }

void USCouleur(uint16_t col)
   {
   uint8_t t[2];
   t[0]=CUSCouleur;
   t[1]=col;
   I2CWriteBytes(I2C_AD_US,t,sizeof(t));
   }
#else
void USStop(void)
   {
   SharedData->detect_av=0;
   SharedData->detect_ar=0;
   }

void USAV(void)
   {
   SharedData->detect_av=1;
   SharedData->detect_ar=0;
   }

void USAR(void)
   {
   SharedData->detect_av=0;
   SharedData->detect_ar=1;
   }

void USAVAR(void)
   {
   SharedData->detect_av=1;
   SharedData->detect_ar=1;
   }

void USAlt(uint16_t mode)
   {
   }

void USCouleur(uint16_t col)
   {
   (void)col;
   }
#endif



volatile unsigned CT_T4;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *tim)
   {
   if(tim==&htim4) CT_T4++;
   }


//Attente tirette en affichant niveau batterie
void BasicWaitTirette(void)
   {
   VTCtrl(1);
   lcdclrscr();

   lcdSetFont("ComicSansMs14");
   const uint16_t dclgn=300;
   int16_t n;
   TickType_t t;
   t=CreateDelaiMs(dclgn);
   n=0;
   while(!_Tirette)
      {
      if(FinDelaiMs(t))
         {
         CloseDelaiMs(t);
         t=CreateDelaiMs(dclgn);
         if(n) lcdTextColor565(C565_Lime);
         else lcdTextColor565(C565_FerrariRed);
         n=!n;
         }
      lcdgotoxy(1,10);
      lcdprintf("Attente tirette...\t");
      DelaiMs(10);
      }
   CloseDelaiMs(t);
   lcdTextColor565(C565_FerrariRed);
   lcdgotoxy(1,10);
   lcdprintf("Tirette en place !\t");

   lcdSetFont("CourierNew12");
   do
      {
      lcdgotoxy(1,12);
      lcdTextColor565(C565_RedWine);
      lcdprintf("Vbat=%.1*1000D\t",VBAT);
      if(VBAT<15000)
         {
         lcdTextColor565(C565_BloodRed);
         lcdprintf(" !!! Faible !!!\t");
         }
      }
   while(_Tirette);
   lcdBackColor565(C565_White);
   lcdTextColor565(C565_Black);
   lcdSetFont("CourierNew12");
   lcdclrscr();
   }

//Attente appui OK sur écran tactile avec affichage niveau batterie
void BasicMenu(void)
{
    lcdSetFont("Consolas28");
    int h=lcdGetPoliceHeight();
    lcdTextColor565(C565_DarkOrchid);
    lcdBackColor565(C565_LightCyan);
    lcdDefTouch(lcdGetPixelWidth()/2-lcdGetTextWidth(" GO "), lcdGetPixelHeight()-1-h, 100," GO ", C565_LightCyan, 0);

    char *s;
    lcdSetFont("Consolas14");
    lcdTextColor565(C565_RoyalBlue);
    while(1)
    {
       s=lcdGetK();
       if(!s) continue;
       if(*s=='A') break;
       lcdgotoxy(1,3); lcdprintf("Vbat=%.2*1000D\t", VBAT);
       //lcdgotoxy(1,4); lcdprintf("Rasp %s\t",CodeErreur?"Ok":"Non Ok");
       DelaiMs(50);
    }
 lcdSetFont("Consolas12");
 lcdBackColor565(C565_White);
 lcdTextColor565(C565_Black);
 lcdclrscr();

}


/*

int fwaitY1000(void)
{
if(P_Y>1000) return 1 ;
return 0 ;
}

void Test1(void)
   {
   InitOdometrie(1500-267,741,2700+7);
   ParamLin(5000,25);
   GMouvement(g_pdistrib);
   GMouvement(g_reposPlat);
   DelaiMs(5000);
   ServoSync(fwaitY1000); //Met la fonction dans la file d’attente
   GMouvement(g_pdistrib);
   GMouvement(g_reposPlat);
   _Powen(1);
   EnableAsserv(1);
   ReculeVers(1500-267,1200);
   AvanceVers(1500-267,741);
   while(1);
   }

void Test2(void)
   {
   uint16_t i;
   _LED_R(0);
   lcdprintf("ça y est, c'est bientôt Noël en été !\n");

   for(i=0; i<5; i++)
      {
      GMouvement(g_pdistrib);
      GMouvement(g_reposPlat);
      }
   DelaiMs(2500);
   ServoStopRequest();
   GMouvement(g_posebalance);
   while(1);
   }
*/



void RT_Main(void *pvParameters)
   {
   SYM_AUTO=0;
    _PowenX(0);
   lcdTaskAllow(xTaskGetCurrentTaskHandle());  //Autorise affichage seulement pour tâche en cours
   Coucou(20,50);
   lcdinit(VT100);
   VTCtrl(1);
      lcdputc(3); //Code init
      lcdputc(3); //Code init
      lcdputc(3); //Code init

      lcdclrscr();lcdclrscr();lcdclrscr();

      VTExtended(1);
      lcdSetFont("Consolas12");
   lcdBackColor565(C565_White);
   lcdTextColor565(C565_Black);
   lcdclrscr();

   lcdprintf("Bonjour\n");


   USCouleur(COULEUR_DEF);
   USAlt(3);

   extern volatile unsigned CT_T4;
   extern volatile uint16_t *const VADC;

   while(0)
      {
      lcdgotoxy(1,5);
      //lcdprintf("CT_ADC=%u  CT_T4=%u ERR_ADC=%u ERR_CODE=%X\t",CT_ADC,CT_T4,ERR_ADC,ERR_CODE);
      lcdprintf("%u %u\t",VADC[0],VADC[1]);
      }

   if(1) BasicMenu();
   if(0) BasicWaitTirette();

//================= Tests initiaux ==================
   if(0) TestFrottementsSecs(300,1,50);

   if(0) TestMoteurs(500,50,500); //TestMoteurs(uint16_t vmax, uint16_t dv, uint16_t dt)
   if(1) TestCodeurs();
   if(0) TestMotX(200,50,500);
   if(0) TestCodeurX();

   if(0) TEST1M();
   if(0) TEST360();
//===================================================

   if(0) { AffPeriphI2C(); CCLeds(0); while(1);}
   if(0) { CCLeds(1); TestCaptCoul();}
   if(1) { CCLeds(0); Test5Servos();}


   //InitSvGr();

//   lcdprintf("ModeRT : %u\n",ServosGetModeRT());
   while(!GetServoReady());

   void ServoTestSend(void);
   //ServoTestSend();



   if(0) {DetectServos(); while(1);} //Affichage des servos sur les bus

   if(0) ServoId(RXID(1), 45); //Reprogrammation de l'ID d'un servo

   SetSvName(SV_EPAULE,"Epaule");
   SetSvName(SV_BRAS,"Bras");
   SetSvName(SV_MAIN,"Main");

   if(0) TestPosServos(3,SV_EPAULE,SV_BRAS,SV_MAIN); //Affichage des positions des servos indiqués

   if(1) TestPosServos(2,AXID(15),AXID(16));

   if(0) {EnableAsserv(1); while(1);}





   /*
    * ************************************************************************
    * ************************** LANCEMENT JEU *******************************
    * ************************************************************************
    */

   if(0) CalageBarillet();

   //GMouvementW(g_reposPlat);

   if(1) { MatchStd(); while(1);}

   VTCtrl(1);

   void (*f)(void);

   f=Menu();
   if(f) f();

   f=GetFctDuringMovement();
   while(1)
      {
      if(f) f();
      //if(TEMPS_RESTANT<-10) LanceFusee();
      }

   }

