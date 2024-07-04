/*Program writen by Maxim Dorokhin https://github.com/m0sBuS
	
	This program written as test for custom merging curcuit board BMW F02 CAN-bus and TOYOTA 3UZ-FE motor.
	
	Using STM32F407VGT MCU, two CAN interfaces TJA1050, one IR2101 for gearbox position motor
	
	Realized features:
		* gearbox and selector CAN connection
				selector messages acknowledge and send gearbox position to BMW
				form BMW gearbox algorithm for Toyota gearbox 
		* gearbox position reading and switching
				soft switch gearbox position with PWM-controlled H-bridge
				reading gearbox position from Toyota gearbox inhibitor
		* reading engine RPM and sending to BMW CAN-bus (rough)
				
	Unrealized features:
		* Presize RPM for dashboard
		* START-STOP button. Starter switching-on and turn-off immediately
		* Addition CAN-bus speed reading from ABS BMW
		* External interrupt for switching
		* Reverse CAN-packet for rear camera
		
	!WARNING! PWM values is high!!!
	If you try start your board with this PWM values you can burn-up your MOSFETs
	Preferably use PWM values less than 50% of your xTIMER->ARR values
	
*/

#include "main.h"

/* Main program */
int main(void)
{
	RCCPLL_init();													//Clock configuration
	GB_Init();															//Gearbox configuration
	CAN_Init();															//CAN-buses configuration
	MS_Timer_Init();												//One millisecond timer timer configuration
	
	if((GPIOD->IDR & 0x1E00) == UNDEFINED)	//Checking gearbox position if position undefined
	{
		GB_Request = DRIVE;										//Switching gearbox to DRIVE
		Braking = 1;													
		GPIOD->ODR &= ~GPIO_ODR_OD4;
		TIM12->CCR1 = 0;
		TIM12->CCR2 = PARKING_PWM;
		GPIOD->ODR |= GPIO_ODR_OD5;
	}
	while(1)
	{
		__WFI();
	}
}

/* Definition clock configuration function */
void RCCPLL_init(void)
{
	RCC->CR |= RCC_CR_HSEON;																						//Switching-on extended clock source
	while (!(RCC->CR & RCC_CR_HSERDY));																	//HSE readiness waiting loop
	RCC->PLLCFGR = 	RCC_PLLCFGR_PLLSRC_HSE | RCC_PLLCFGR_PLLQ_3 |				//Configuring PLL for use HSE clock source and peripherial bus clocks (64MHz)
									RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLM_3; 
	RCC->CR |= RCC_CR_PLLON;																						//Switching-on PLL module
	while (!(RCC->CR & RCC_CR_PLLRDY));																	//PLL readiness waiting loop
	RCC->CFGR = /*RCC_CFGR_PPRE1_2 |*/ RCC_CFGR_SW_PLL;									//Switching system clock source on PLL
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);							//Sysclock readiness waiting loop
	RCC->CR &= ~RCC_CR_HSION;																						//Switching-off internal clock source
}

/* Definition millisecond timer configuration function */
void MS_Timer_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;																	//Switching-on timer 9
	
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);																	//Switching-on timer 9 interrupt
	
	TIM9->CNT = 0;																											//Reset counter
	TIM9->PSC = 31;																											//Prescaler for 1 MHz timer clock
	TIM9->ARR = 999;																										//Coutner period for 1 ms
	TIM9->DIER = TIM_DIER_UIE;																					//Timer update interrupt
	TIM9->CR1 = TIM_CR1_ARPE;																						//Configure timer 4 for auro-reload preload
	TIM9->CR1 |= TIM_CR1_CEN;																						//Timer switching-on
}

/* Definition millisecond timer interrupt function */
void TIM1_BRK_TIM9_IRQHandler(void)
{
	TIM9->SR = 0;
	GB_Braking();
	CANGearPosition100ms++;
	CANReversePacket2s++;
	if (CANGearPosition100ms == 100)
	{
		if (HardReset == 1)
		{
			CAN_GBPos(GB_Status);
			CANGearPosition100ms = 0;
		}		
	}
	if (CANReversePacket2s == 2000)
	{
		if (!startPump)
		{
			CAN_TxMessage[4].DATA[1] = 0x00;
			startPump = 1;
		}
		CANReversePacket2s = 0;
	}
	if (CANGearPosition100ms == 80 && HardReset == 1)
		CAN_Message_Send(0);
	if (CANGearPosition100ms == 70 && HardReset == 1)
		CAN_Message_Send(4);
	if (CANGearPosition100ms == 60 && HardReset == 1)
		BMW_CAN_Message_Send(5, 0x7A, 0x7);
	if (CANGearPosition100ms == 50 && HardReset == 1)
		BMW_CAN_Message_Send(6, 0x9F, 0x7);
	if (CANGearPosition100ms == 40 && HardReset == 1)
		BMW_CAN_Message_Send(7, 0xF1, 0x7);
	if (CANGearPosition100ms == 30 && HardReset == 1)
		BMW_CAN_Message_Send(8, 0xB2, 0x3);
}
