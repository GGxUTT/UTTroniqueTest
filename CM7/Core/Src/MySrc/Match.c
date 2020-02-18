/*
 * Match.c
 *
 *  Created on: 28 nov. 2016
 *      Author: Robot
 */
//#include "stm32f7xx_hal.h"

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



#include <math.h>

#include "Match.h"
#include "ServoDefs.h"
#include "CommandesServos.h"
#include "Sequenceur.h"
#include "Odometrie.h"
#include "spidma.h"
#include "TempsMs.h"
#include "LCD_VT100.h"
#include "CommandesServos.h"
#include "I2CMaitre.h"
#include "I2CUSCommun.h"
#include "Config.h"
#include "LowLevel.h"
#include "Menu.h"
#include "Communic.h"
#include "Asserv.h"
#include "Dijkstra.h"

#define DBG 0

#define P_X0 (1500-267)
#define P_Y0 137


static uint16_t FINALE=0;


//Définition de la distance de sécurité entre les robots (bras relevés), en mm. Mettre 0 pour contact.
void USDist(int16_t mm)
   {
   const int16_t d_av=120, d_ar=110;  //Distances séparant le capteur US % bords du robot
   const int16_t d_adv=150;  //Distance séparant la balise du bord du robot adverse (estimation moyenne)
   uint8_t t[2];
   if(mm<=0)  //US off
      {
      t[0]=CUSDistAV;
      t[1]=5;
      I2CWriteBytes(I2C_AD_US,t,sizeof(t));
      t[0]=CUSDistAR;
      I2CWriteBytes(I2C_AD_US,t,sizeof(t));
      return;
      }
   mm+=d_adv;
   t[0]=CUSDistAV;
   t[1]=(mm+d_av)*0.1f;
   I2CWriteBytes(I2C_AD_US,t,sizeof(t));
   t[0]=CUSDistAR;
   t[1]=(mm+d_ar)*0.1f;
   I2CWriteBytes(I2C_AD_US,t,sizeof(t));
   }



void FinMatch(void)
   {
   Pompe(0);
   }

#define ParamLinStd() ParamLin(30000,30)
#define ParamRotStd() ParamRot(20000,25)
#define USDistNorm() USDist(200)

#define WAIT 0x80


void Pompe(int st) //-1 : aspire   0 : Arret   1 : souffle
   {
   if(st>0) {_PompeA(1); _PompeB(0); _PowPompe(1); return;}
   if(st<0) {_PompeA(0); _PompeB(1); _PowPompe(1); return;}
   _PowPompe(0);
   }


void PompeSouffle(void)
{
	Pompe(1);
	DelaiMs(200);
}

void PompeAspire(void)
{
	Pompe(-1);
}

void PompeArret(void)
{
	Pompe(0);
}



void AffOdo(void)
   {
   lcdgotoxy(1,2);
   lcdprintf("X=%d Y=%d\t",P_X,P_Y);
   lcdgotoxy(1,3);
   lcdprintf("A=%.1*10D\t",THETA);
   lcdgotoxy(1,4);
   lcdprintf("Dist=%.1f\t",VDist);

   lcdgotoxy(1,6);
   lcdprintf("Vbat=%.1*1000D  -  Reste %.1*10D\t",VBAT,RBT_TIME);
   lcdgotoxy(1,7);
   lcdprintf("Mes Points=%u\t",GetGros()->score);

   }


void MatchQualif(void)
   {
   MatchStd();
   }

void MatchFinale(void)
   {
   FINALE=1;
   MatchStd();
   }

void TestDijkstra(int16_t x0, int16_t y0, int16_t xd, int16_t yd)
{
	   TickType_t t;
	   int r;
	   uint16_t i,nb;
	   InitOdometrie(x0,y0,0);
	   lcdprintf("(%d,%d)->(%d,%d) :\n",x0,y0,xd,yd);
	   t=xTaskGetTickCount();
	   _LEDV(1);
	   r=DijkRoadTo(xd,yd);
	   _LEDV(0);
	   t=xTaskGetTickCount()-t;
	   lcdprintf("Longueur : %d - Calcul : %dms",r,t);
	   nb=DijkGetNb();
	   for(i=0; i<nb; i++)
	      {
		   int16_t x,y;
		  DijkReadPoint(i,&x,&y);
	      if(!(i%3)) lcdprintf("\n");
	      lcdprintf("(%d,%d) ",x,y);
	      }
	   lcdprintf("\n------------\n");


}


int DijkstraAvanceVers(int16_t xd, int16_t yd)
{
	int r;
	uint16_t i,nb;
	int16_t x,y;
	do
	{
		r=DijkRoadTo(xd,yd);
		if(!r) { StopUrgence(); return 0; } //Pas de chemin trouvé
		nb=DijkGetNb();
		for(i=1; i<nb; i++)
		{
			DijkReadPoint(i,&x,&y);
			SiUrgence(50,1000);
			r=AvanceVers(x,y);
			if(r!=RBT_EOK) break;
		}
	}while(i!=nb);
	StopUrgence();
	return 1;
}

void MatchStd(void)
   {

   lcdclrscr();
   lcdprintf("Go...");
   VTCtrl(0);

   if(COULEUR==COUL_POS) InitOdometrie(P_X0,741,2700+7); //2707
   else InitOdometrie(-(P_X0),753,900-7);

   _Powen(1);
   EnableAsserv(1);
   VTCtrl(1);

   DelaiMs(10);
   USDist(100);
   ParamLin(20000,25);
   DefineFctDuringMovement(AffOdo);

   TestDijkstra(-1400,200,1400,200);
   TestDijkstra(-800,1200,800,1200);
   //DijkAddObstacle(0,1400,200,5000);
   TestDijkstra(-800,1200,800,1200);
   TestDijkstra(-800,200,800,1400);

   while(1);

   return;
   }





void AffHomolog(void)
   {

   lcdgotoxy(1,4);
   lcdprintf("Mes Points=%u\t",GetGros()->score);

   lcdgotoxy(1,5);
   lcdprintf("Vbat=%.1*1000D V\t",VBAT);
   lcdgotoxy(1,6);
   lcdprintf("Reste %.1*10D s\t",RBT_TIME);
   }


void Homologation(void)
   {
   lcdclrscr();
   lcdprintf("Homologation...");

   if(COULEUR==COUL_POS) InitOdometrie(P_X0,741,2700+7); //2707
   else InitOdometrie(-(P_X0),753,900-7);

   _Powen(1);
   EnableAsserv(1);
   VTCtrl(1);

   DelaiMs(10);
   ParamLin(10000,25);
   DefineFctDuringMovement(AffHomolog);

   }



void Demo(void)
   {
   DefineEndOfGame(FinMatch);
   lcdclrscr();
   lcdprintf("Démo...\n");
#if 0
   lcdprintf("Attente Tirette...\n");
   while(!_Tirette) DelaiMs(10);
   while(_Tirette);
#endif

   if(COULEUR==COUL_POS) InitOdometrie(P_X0,741,2700+7); //2707
   else InitOdometrie(-(P_X0),753,900-7);

   _Powen(1);
   EnableAsserv(1);
   VTCtrl(1);


   }


