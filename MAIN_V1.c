#include "RTE_Device.h"                 // Keil::Device:Startup
#include "GPIO_LPC17xx.h"               // Keil::Device:GPIO
#include "Driver_USART.h"               // ::CMSIS Driver:USART

extern ARM_DRIVER_USART Driver_USART0;

void PWM_Lidar(void);
void Init_UART(void);

char tabT[2]={0xA5,0x20};
char tabR[5];
int i;

int main(void)
{
    PWM_Lidar();
    Init_UART();

    
    while (1){
        while(Driver_USART0.GetStatus().tx_busy == 2); // attente buffer TX vide
        for(i=0;i<10000;i++);
        Driver_USART0.Send(tabT,2);
       
        Driver_USART0.Receive(tabR,5);
    }
    return 0;
}


void PWM_Lidar(void) // 25kHz; PWM:60%; P2.5
{
    LPC_SC->PCONP = LPC_SC->PCONP | (1<<6);
    
    LPC_PINCON->PINSEL4 |= (1<<10);     //PWM 1.6   
    LPC_PWM1->CTCR = 0;                                // timer
    LPC_PWM1->PR = 0;                                    //PR=0
    LPC_PWM1->MR0 = 999;                            //25kHz
    LPC_PWM1->MCR |= (1<<1);                    //RAZ du compteur si correspondance avec MR0   
    LPC_PWM1->PCR |= (1<<14);                    //start PWM
    LPC_PWM1->MR6 = 599;                            //PWM 60%
    LPC_PWM1->TCR =1;                                    //start timer
}


void Init_UART(void){
    Driver_USART0.Initialize(NULL);
    Driver_USART0.PowerControl(ARM_POWER_FULL);
    Driver_USART0.Control(    ARM_USART_MODE_ASYNCHRONOUS |
                            ARM_USART_DATA_BITS_8        |
                            ARM_USART_PARITY_NONE        |
                            ARM_USART_FLOW_CONTROL_NONE,
                            115200);
    Driver_USART0.Control(ARM_USART_CONTROL_TX,1);
    Driver_USART0.Control(ARM_USART_CONTROL_RX,1);
}
