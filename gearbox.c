#include "gearbox.h"

void GB_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
	GPIOB->MODER |= GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15);
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_3 | GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_3;
	
	GPIOD->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE8 |
										GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 |
										GPIO_MODER_MODE12);
	GPIOD->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	
	TIM12->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |				//Configure timer 12 for generate positive PWM signal for 2 output channels
								TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 ;
	TIM12->PSC = 0;																							//Timer 12 counter prescaler is 0, for high frequency PWM signal
	TIM12->ARR = 3199;																					//Timer 12 counter period (32 MHz / 3200 = 10 kHz PWM)
	TIM12->CCR1 = 0;																						//Timer 12 start duty cycle for 1st PWM channel
	TIM12->CCR2 = 0;																						//Timer 12 start duty cycle for 2nd PWM channel
	TIM12->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM12->CR1 = TIM_CR1_ARPE;																	/*Configure timer 12 for auro-reload preload (if CCR12 or another will be changed 
																															then won't work immediately, but will work after PWM cycle)*/
	TIM12->CR1 |= TIM_CR1_CEN;																	//Switching-on timer 12
}

void CAN_GBPos(enum GB_Pos_enum GB_Pos)
{
	switch(GB_Pos)
	{
		case PARKING:
			CAN_TxMessage[2].DATA[2] = 0x20;
		break;
		
		case REVERSE:
			CAN_TxMessage[2].DATA[2] = 0x40;
		break;
	
		case NEUTRAL:
			CAN_TxMessage[2].DATA[2] = 0x60;
		break;
		
		case DRIVE:
//			if (GPIOA->IDR & BREAK_POS)
//				CAN_TxMessage[2].DATA[2] = 0x81;
//			else
				CAN_TxMessage[2].DATA[2] = 0x80;
		break;
						
		case MANUAL:
			CAN_TxMessage[2].DATA[2] = 0x40;
		break;
		
		case UNDEFINED:
			
		break;
	}
	CAN_TxMessage[2].DATA[3] = 0x00;
	CAN_TxMessage[2].DATA[4] = 0x00;
	BMW_CAN_Message_Send(2,0xD6,4);
}

void GB_CANRequest(void)
{
	if (GPIOD->IDR & BREAK_POS /* && ������� ������� �������� */)
	{
//	if (PrevPacket != CAN_SwMessage.DATA[2])
//	{
//		PacketCNT = 0;
//		PrevPacket = CAN_SwMessage.DATA[2];
//	}
//	else
//		PacketCNT++;
	if (CAN_SwMessage.DATA[3] == 0xD5)
		GB_Request = PARKING;
//	if (PacketCNT == 25)
//	{
	switch (CAN_SwMessage.DATA[2])
	{
		case 0x0F:
			OneSwitch = 0;
		break;
						
		case 0x1F:
			if (OneSwitch == 0)
			{
				switch(GB_Request)
				{
					case PARKING:
						GB_Request = NEUTRAL;
					break;
									
					case REVERSE:
						GB_Request = PARKING;
					break;
									
					case NEUTRAL:
						GB_Request = REVERSE;
					break;
									
					case DRIVE:
						GB_Request = NEUTRAL;
					break;
									
					case MANUAL:
						// Impossible case //
					break;
									
					case UNDEFINED:
						// Impossible case //
					break;
				}
				OneSwitch = 1;
			}
		break;
						
		case 0x2F:
			GB_Request = REVERSE;
		break;
						
		case 0x3F:
			if (OneSwitch == 0)
			{
				switch(GB_Request)
				{
					case PARKING:
						GB_Request = NEUTRAL;
					break;
									
					case REVERSE:
						GB_Request = NEUTRAL;
					break;
									
					case NEUTRAL:
						GB_Request = DRIVE;
					break;
									
					case DRIVE:
						// Impossible case //
					break;
									
					case MANUAL:
						// Impossible case //
					break;
												
					case UNDEFINED:
						// Impossible case //			
					break;
				}
				OneSwitch = 1;
			}
		break;
						
		case 0x4F:
			GB_Request = DRIVE;
		break;
						
		case 0x7F:
			GB_Request = MANUAL;
			OneSwitch = 0;
		break;
						
		case 0x5F:
			if (OneSwitch == 0)
			{
				// Manual switch up gear //
				OneSwitch = 1;
			}
		break;
							
		case 0x6F:
			if (OneSwitch == 0)
			{
				// Manual switch down gear //
				OneSwitch = 1;
			}
		break;
		}
//				}
	}
}

void GB_Position(void)
{
	Timeout = 20;
	while (GB_Status != GB_Request)
	{
		if (TIM12->CNT == 3199) 
		{
			if (!CNT_Reset)
			{
				CNT_Reset = 1;
				Timeout--;
			}
		}
		else 
			CNT_Reset = 0;
		if (!Timeout)
		{
			GPIOD->ODR &= ~(GPIO_ODR_OD4 | GPIO_ODR_OD5);
			TIM12->CCR1 = 0;
			TIM12->CCR2 = 0;	
			break;
		}
		switch(GPIOD->IDR & 0x1E00)
		{
			case P_POS:
				GB_Status = PARKING;
			break;
			
			case R_POS:
				GB_Status = REVERSE;
			break;
			
			case N_POS:
				GB_Status = NEUTRAL;
			break;
			
			case D_POS:
				GB_Status = DRIVE;
			break;
			
			default:
				GB_Status = UNDEFINED;
			break;
		}
		if (GB_Status == REVERSE)
			CAN_TxMessage[0].DATA[0] = 0xFE;
		else
			CAN_TxMessage[0].DATA[0] = 0xFD;
		switch(GB_Status)
		{
			case PARKING:
				switch(GB_Request)
				{
					case PARKING:
						GB_Status = GB_Request;
						Braking = BRAKE_DELAY;
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = BRAKE_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case REVERSE:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
					
					case NEUTRAL:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
						
					case DRIVE:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
					
					case UNDEFINED:
						
					break;
					
					case MANUAL:
						
					break;
				}
			break;
				
			case REVERSE:
				switch(GB_Request)
				{
					case PARKING:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = PARKING_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case REVERSE:
						GB_Status = GB_Request;
						Braking = BRAKE_DELAY;
						if (GPIOD->ODR == GPIO_ODR_OD4)
						{
							GPIOD->ODR &= ~GPIO_ODR_OD4;
							TIM12->CCR1 = 0;
							TIM12->CCR2 = BRAKE_PWM;
							GPIOD->ODR |= GPIO_ODR_OD5;
						}
						if (GPIOD->ODR == GPIO_ODR_OD5)
						{
							GPIOD->ODR &= ~GPIO_ODR_OD5;
							TIM12->CCR1 = BRAKE_PWM;
							TIM12->CCR2 = 0;
							GPIOD->ODR |= GPIO_ODR_OD4;
						}
					break;
					
					case NEUTRAL:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
						
					case DRIVE:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
										
					case UNDEFINED:
						
					break;
					
					case MANUAL:
						
					break;
				}
			break;
				
			case NEUTRAL:
				switch(GB_Request)
				{
					case PARKING:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = SWITCH_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case REVERSE:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = SWITCH_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case NEUTRAL:
						GB_Status = GB_Request;
						Braking = BRAKE_DELAY;
						if (GPIOD->ODR == GPIO_ODR_OD4)
						{
							GPIOD->ODR &= ~GPIO_ODR_OD4;
							TIM12->CCR1 = 0;
							TIM12->CCR2 = BRAKE_PWM;
							GPIOD->ODR |= GPIO_ODR_OD5;
						}
						if (GPIOD->ODR == GPIO_ODR_OD5)
						{
							GPIOD->ODR &= ~GPIO_ODR_OD5;
							TIM12->CCR1 = BRAKE_PWM;
							TIM12->CCR2 = 0;
							GPIOD->ODR |= GPIO_ODR_OD4;
						}
					break;
						
					case DRIVE:
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = SWITCH_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
					
					case UNDEFINED:
						
					break;
					
					case MANUAL:
						
					break;
				}
			break;
				
			case DRIVE:
				switch(GB_Request)
				{
					case PARKING:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = SWITCH_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case REVERSE:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = SWITCH_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
					
					case NEUTRAL:
						GPIOD->ODR &= ~GPIO_ODR_OD4;
						TIM12->CCR1 = 0;
						TIM12->CCR2 = SWITCH_PWM;
						GPIOD->ODR |= GPIO_ODR_OD5;
					break;
						
					case DRIVE:
						GB_Status = GB_Request;
						Braking = BRAKE_DELAY;
						GPIOD->ODR &= ~GPIO_ODR_OD5;
						TIM12->CCR1 = BRAKE_PWM;
						TIM12->CCR2 = 0;
						GPIOD->ODR |= GPIO_ODR_OD4;
					break;
					
					case UNDEFINED:
						
					break;
					
					case MANUAL:
						
					break;					
				}
			break;
				
			case UNDEFINED:
						
			break;
			
			case MANUAL:
				
			break;
		}
	}
	if (Braking)
	{
		if (TIM12->CNT == 3199) 
		{
			if (!CNT_Reset)
			{
				CNT_Reset = 1;
				Braking--;
			}
		}
		else 
			CNT_Reset = 0;
	}
	else
	{
		GPIOD->ODR &= ~(GPIO_ODR_OD4 | GPIO_ODR_OD5);
		TIM12->CCR1 = 0;
		TIM12->CCR2 = 0;
	}
}
