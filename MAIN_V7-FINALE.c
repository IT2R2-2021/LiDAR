#include "RTE_Device.h"                 // Keil::Device:Startup
#include "GPIO_LPC17xx.h"               // Keil::Device:GPIO
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "LPC17xx.h"                    // Device header
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "stdio.h"
#include "stdlib.h"
#include "GLCD_Config.h"                // Keil.MCB1700::Board Support:Graphic LCD
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "Driver_SPI.h"                 // ::CMSIS Driver:SPI
#include <string.h>
#include <math.h>

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

osThreadId ID_Reception;
osThreadId ID_Traitement;


char tabR[10];
char tabT[2]={0xA5,0x20};

char LCD[50];

char tabAngle90[20];
char tabAngle[20];
char tabDist[20];
char gigatab[50];
char tabQual[20];
char tabMoyenne[20];
char Send[50];

void PWM_Lidar(void);
void Init_UART(void);
void Init_UART1(void);
void Reception(void const* arg);
void Traitement(void const* arg);

osThreadDef(Reception,osPriorityNormal,1,0);
osThreadDef(Traitement,osPriorityNormal,1,0);


int main (void){

	PWM_Lidar();
	Init_UART();
	Init_UART1();	
	GLCD_Initialize();
	GLCD_ClearScreen();
	GLCD_SetFont(&GLCD_Font_16x24);	
	osKernelInitialize();
	
	while(Driver_USART0.GetStatus().tx_busy == 1); 	// attente buffer TX vide
	Driver_USART0.Send((const void*)tabT,2);				// envoie de la commande Scan "A5,20"
		
		
	ID_Reception=osThreadCreate(osThread(Reception),NULL);
	ID_Traitement=osThreadCreate(osThread(Traitement),NULL);	


	osKernelStart();

	osDelay(osWaitForever);
	
return 0;
}

void Reception(void const* arg){

	int i=0;
	short angle1, angle2, angle, dist1, dist2, distance, qualite;
	
	while(Driver_USART0.GetStatus().tx_busy == 1); 	// attente buffer TX vide
	Driver_USART0.Send((const void*)tabT,2);				// envoie de la commande Scan "A5,20"
	
	while(1){

	Driver_USART0.Receive(tabR,10);								//reception RX des trames du LiDAR
		
		while(Driver_USART0.GetRxCount() < 1);				
			for (i=0;i<5;i++){						
				if (((tabR[i]&0x3E)==0x3E)&&((tabR[i+1]&0x01)==0x01)){		//masquage des trames de mauvaise qualité et du checksum	
						
						//sprintf(LCD,"%02x %02x %02x %02x %02x",tabR[i],tabR[i+1],tabR[i+2],tabR[i+3],tabR[i+4]);																//Affichage sans traitement des octets(qualité - angle_1 - angle_2 - distance_1 - distance_2 )
						//GLCD_DrawString(0, 0, LCD);
						qualite=tabR[i];
						angle1=tabR[i+1];																						//placement des octets dans des variables de type short
						angle2=tabR[i+2];
						dist1=tabR[i+3];
						dist2=tabR[i+4];
						
						angle = ((((angle2 << 8)|angle1)>>1)&0x7FFF)>>6;			//décalage et mise en forme des octets d'angle pour avoir qqch d'utile, division par 64 pour le traitement
						distance = ((dist2<<8) | dist1) >>2;													//décalage et mise en forme des octets de distance pour avoir qqch d'utile, division par 4 pour le traitement
					
						distance = distance/10;					
//						sprintf(tabDist,"angle %3d dist %5d",angle, distance);
//						GLCD_DrawString(0, 0, tabDist);
					
		if (distance>200)
			{
				distance=200;																						//limite la valeur à 2m
			}
		
			
			if (distance<=200 && distance>=20 && angle>=0 && angle <=360)
			{
			sprintf(tabDist,"dist %5d cm    ", distance);								//Affichage DISTANCE HT					
			while(Driver_USART1.GetStatus().tx_busy == 1); 								// attente buffer TX vide
			Driver_USART1.Send((const void*)tabDist,strlen(tabDist));

			sprintf(tabAngle,"angle %3d \n\r", angle);										//Affichage ANGLE HT					
			while(Driver_USART1.GetStatus().tx_busy == 1); 								// attente buffer TX vide
			Driver_USART1.Send((const void*)tabAngle,strlen(tabAngle));
			}
			
			
	if(angle>0 && angle <=45){																			//angle de 0° à 45°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
//		sprintf(tabDist,"angle %3d dist %5d",angle, distance);
//		GLCD_DrawString(0, 0, tabDist);
			//
			//instructions
			//
		}
	}

	if(angle>45 && angle <=90){																			//angle de 45° à 90°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>90 && angle <=135){																			//angle de 90° à 135°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>135 && angle <=180){																			//angle de 135° à 180°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>180 && angle <=225){																			//angle de 180° à 225°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>225 && angle <=270){																			//angle de 225° à 270°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>270 && angle <=315){																			//angle de 270° à 315°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
	if(angle>315 && angle <=360){																			//angle de 315° à 360°
		if(distance>20 && distance <=200)														//distance entre 20cm et 2m
		{
			//
			//instructions
			//
		}
	}
				}	
		}				
			osDelay(500);
	}
}



void Traitement (void const* arg){

}

void PWM_Lidar(void){ // 25kHz; PWM:60%; P2.5
	LPC_SC->PCONP = LPC_SC->PCONP | (1<<6);
	
	LPC_PINCON->PINSEL4 |= (1<<10); 	//PWM 1.6	
	LPC_PWM1->CTCR = 0;								// timer
	LPC_PWM1->PR = 0;									//PR=0
	LPC_PWM1->MR0 = 999;							//25kHz
	LPC_PWM1->MCR |= (1<<1);					//RAZ du compteur si correspondance avec MR0	
	LPC_PWM1->PCR |= (1<<14);					//start PWM
	LPC_PWM1->MR6 = 599;							//PWM 60%
	LPC_PWM1->TCR =1;									//start timer
}


void Init_UART(void){
	Driver_USART0.Initialize(NULL);
	Driver_USART0.PowerControl(ARM_POWER_FULL);
	Driver_USART0.Control(	ARM_USART_MODE_ASYNCHRONOUS |
													ARM_USART_DATA_BITS_8				|
													ARM_USART_STOP_BITS_1				|
													ARM_USART_PARITY_NONE				|
													ARM_USART_FLOW_CONTROL_NONE,
													115200);
	Driver_USART0.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART0.Control(ARM_USART_CONTROL_RX,1);
}

void Init_UART1(void){																					//HyperTerminal
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |
													ARM_USART_DATA_BITS_8				|
													ARM_USART_STOP_BITS_1				|
													ARM_USART_PARITY_NONE				|
													ARM_USART_FLOW_CONTROL_NONE,
													115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);
}

