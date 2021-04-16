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

	
char tabR[10];
char tabT[2]={0xA5,0x20};

char LCD[50];

char tabAngle[20];
char tabDist[20];
char gigatab[50];

char Send[50];

void PWM_Lidar(void);
void Init_UART(void);
void Init_UART1(void);

int main(void)
{ 
	short angle, distance, angle1, angle2, dist1, dist2;
	int i=0, x = 0, y = 0;
	PWM_Lidar();
	Init_UART();
	Init_UART1();
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
				if (((tabR[i]&0x3E)==0x3E)&&((tabR[i+1]&0x01)==0x01)){					//masquage des trames de mauvaise qualité et du checksum
					if (((tabR[i]&0x3E)==0x3E)&&((tabR[i+1]&0x01)==0x01)){	
						
						sprintf(LCD,"%02x %02x %02x %02x %02x",tabR[i],tabR[i+1],tabR[i+2],tabR[i+3],tabR[i+4]);																//Affichage sans traitement des octets(qualité - angle_1 - angle_2 - distance_1 - distance_2 )
						GLCD_DrawString(0, 0, LCD);
						
//						sprintf(LCD,"%0x",tabR[i+1]);
//						GLCD_DrawString(60, 0, LCD);
//						
//						sprintf(LCD,"%0x",tabR[i+2]);
//						GLCD_DrawString(120, 0, LCD); 
//						
//						sprintf(LCD,"%0x",tabR[i+3]);
//						GLCD_DrawString(180, 0, LCD);
//						
//						sprintf(LCD,"%0x",tabR[i+4]);
//						GLCD_DrawString(240, 0, LCD); 
						
						
						angle1=tabR[i+1];																						//placement des octets dans des variables de type short
						angle2=tabR[i+2];
						dist1=tabR[i+3];
						dist2=tabR[i+4];
						
						
						angle = ((((angle2 << 8)|angle1)>>1)&0x7FFF)>>6;							//décalage et mise en forme des octets d'angle pour avoir qqch d'utile, division par 64 pour le traitement
						distance = ((dist2<<8) | dist1) >>2;													//décalage et mise en forme des octets de distance pour avoir qqch d'utile, division par 4 pour le traitement
						if (distance>2000)
							{
								distance=2000;
							}
					
					if((angle>315 && angle <=359)||(angle>0 && angle <=45)){

						if(distance>200 && distance <=2000)
						{
							sprintf(tabDist,"dist %5d mm ", distance);
							//GLCD_DrawString(0,60,tabDist);																           //affichage distance
								
							while(Driver_USART1.GetStatus().tx_busy == 1); 										        	// attente buffer TX vide
							Driver_USART1.Send((const void*)tabDist,strlen(tabDist));										// envoi + affichage HyperTerminal

						}
						
						sprintf(tabAngle,"angle %04d\n\r", angle);											
						//GLCD_DrawString(0,30,tabAngle);																					  	//affichage angle
						
						while(Driver_USART1.GetStatus().tx_busy == 1); 															  // attente buffer TX vide
						Driver_USART1.Send((const void*)tabAngle,strlen(tabAngle));									  // envoi + affichage HyperTerminal

					}
						x = cos(angle)*(distance/20);
						y = sin(angle)*(distance/20);
						x=x+160;
						y=y+120;
						
						
						
						
	//					GLCD_DrawPixel(x, y);
	//					if ((distance>200 & distance<500)&&(angle>0 && angle<45))
	//					{
	//					GLCD_DrawString(0,90,"1");
	//					}
	//					else
	//					{
	//					GLCD_DrawString(0,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>45 && angle<90))
	//					{
	//					GLCD_DrawString(20,90,"2");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(20,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>90 && angle<135))
	//					{
	//					GLCD_DrawString(40,90,"3");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(40,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>135 && angle<180))
	//					{
	//					GLCD_DrawString(60,90,"4");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(60,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>180 && angle<225))
	//					{
	//					GLCD_DrawString(80,90,"5");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(80,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>270 && angle<315))
	//					{
	//					GLCD_DrawString(100,90,"6");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(100,90," ");
	//					}
	//					
	//					if ((distance>200 & distance<500)&&(angle>315 && angle<360))
	//					{
	//					GLCD_DrawString(120,90,"7");							
	//					}
	//					else
	//					{
	//					GLCD_DrawString(120,90," ");
	//					}
//							
					
				}
			}
		}
	}
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

void Init_UART1(void){
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
