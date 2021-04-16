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

extern ARM_DRIVER_USART Driver_USART0;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

	
char tabR[10];
char tabT[2]={0xA5,0x20};

char LCD[10];

char tabAngle[2];
char tabDist[2];

char Send[50];

void PWM_Lidar(void);
void Init_UART(void);


int main(void)
{	
	short angle, distance, angle1, angle2, dist1, dist2;
	int i;
	PWM_Lidar();
	Init_UART();
	GLCD_Initialize();
	GLCD_ClearScreen();
	GLCD_SetFont(&GLCD_Font_16x24);
	
	
	
	while(Driver_USART0.GetStatus().tx_busy == 1); 	// attente buffer TX vide
	Driver_USART0.Send((const void*)tabT,2);				// envoie de la commande Scan "A5,20"
	
	
	while(1)
	{
		Driver_USART0.Receive(tabR,10);								//reception RX des trames du LiDAR
		while(Driver_USART0.GetRxCount() < 1);				
			for (i=0;i<5;i++){						
				if (((tabR[i]&0x3E)==0x3E)&&((tabR[i+1]&0x01)==0x01)){			//masquage des trames de mauvaise qualité et du checksum
					
					sprintf(LCD,"%0x",tabR[i]);																//Affichage sans traitement des octets(qualité - angle_1 - angle_2 - distance_1 - distance_2 )
					GLCD_DrawString(0, 0, LCD);
					
					sprintf(LCD,"%0x",tabR[i+1]);
					GLCD_DrawString(60, 0, LCD);
					
					sprintf(LCD,"%0x",tabR[i+2]);
					GLCD_DrawString(120, 0, LCD); 
					
					sprintf(LCD,"%0x",tabR[i+3]);
					GLCD_DrawString(180, 0, LCD);
					
					sprintf(LCD,"%0x",tabR[i+4]);
					GLCD_DrawString(240, 0, LCD); 
					
					
					angle1=tabR[i+1];																						//placement des octets dans des variables de type short
					angle2=tabR[i+2];
					dist1=tabR[i+3];
					dist2=tabR[i+4];
					
					
					angle = ((((angle2 << 8)|angle1)>>1)&0x7FFF)>>6;							//décalage et mise en forme des octets d'angle pour avoir qqch d'utile, division par 64 pour le traitement
					distance = ((dist2<<8) | dist1) >>2;													//décalage et mise en forme des octets de distance pour avoir qqch d'utile, division par 4 pour le traitement
					
					sprintf(tabAngle,"angle %04d", angle);											
					GLCD_DrawString(0,100,tabAngle);															//affichage angle
					sprintf(tabDist,"dist %5d mm", distance);
					GLCD_DrawString(0,150,tabDist);																//affichage distance
				}
			}
		}
	
	return 0;
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
