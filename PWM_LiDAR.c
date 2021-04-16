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
