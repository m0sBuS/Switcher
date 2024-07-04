/*Program writen by Maxim Dorokhin https://github.com/m0sBuS
	
	This program written for custom merging curcuit board BMW F02 CAN-bus and TOYOTA 3UZ-FE motor.
	
	This program describe whole CAN-buses lines and BMW,Toyota CAN messages
*/

#include "CAN.h"

/* Definition CAN-bus hard reset function (Must be removed for useless) */ 
void CAN_HardReset(void)
{
	for (uint32_t i = 0; i <= 0xC; i += 4)
	{
		uint32_t u32var = *(uint32_t*)((uint32_t)&CAN_TxMessage[1] + i);
		*(uint32_t*)((uint32_t)&CAN2->sTxMailBox[0] + i) = u32var;
	}
	while (CAN2->MSR & CAN_MSR_STATUSMASK);
	CAN2->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
	HardReset = 1;
}

/* Definition CAN-buses initialization function */
void CAN_Init(void)
{
	uint16_t Timeout = 0xFFFF;																							//Timeout counter for CAN initializers 
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;							//Switching-on GPIOA and GPIOB
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN;								//Switching-on CAN1 and CAN2 interfaces
	
	GPIOA->MODER &= ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12);								//Configure PA11 and PA12 for Alternate-Function push-pull outputs 
	GPIOA->MODER |= GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12);
	GPIOA->AFR[1] |= 	GPIO_AFRH_AFSEL11_0 | GPIO_AFRH_AFSEL11_3 |
										GPIO_AFRH_AFSEL12_0 | GPIO_AFRH_AFSEL12_3;
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13);								//Configure PB12 and PB13 for Alternate-Function push-pull outputs 
	GPIOB->MODER |= GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13;
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL12 | GPIO_AFRH_AFSEL13);
	GPIOB->AFR[1] |= 	GPIO_AFRH_AFSEL12_0 | GPIO_AFRH_AFSEL12_3 | 
										GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_3;

	NVIC_EnableIRQ(CAN1_RX1_IRQn);																					//Switching-on CAN-bus interrupts
	NVIC_EnableIRQ(CAN1_TX_IRQn);
	NVIC_EnableIRQ(CAN2_RX1_IRQn);
	NVIC_EnableIRQ(CAN2_TX_IRQn);
	
	CAN1->MCR = CAN_MCR_RESET;																							//Reset Toyota CAN-bus interface
	while((CAN1->MSR & CAN_MCR_RESET) && Timeout != 0)											//Reset readiness waiting loop
	{
		Timeout--;
	}
	if (Timeout == 0)
		CAN_init_Error = true;
	CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_NART | CAN_MCR_AWUM;									//CAN init, no automatic retransmitiion, automatic wakeup mode
	CAN1->IER = CAN_IER_FMPIE1 | CAN_IER_FOVIE1 | CAN_IER_TMEIE;						//CAN FIFO message pending, overrun, transmit interrupt enable
	CAN1->BTR = 0x001c0003;																									//Baudrate for 500kbps CAN
	CAN1->MCR &= ~((uint32_t)CAN_MCR_INRQ);																	//End CAN configuration 
	Timeout = 0xFFFF;																												//Reinit tieout counter
	while((CAN1->MSR&CAN_MSR_INAK) && Timeout != 0)													//Waiting CAN initialization
	{
		Timeout--;
	}
	if (Timeout == 0)
		CAN_init_Error = true;
	CAN1->FMR = CAN_FMR_FINIT;																							//CAN filter initialization configuration
	CAN1->FA1R = CAN_FA1R_FACT10;																						//Filter activation (I use 10th filter)
	CAN1->FFA1R = CAN_FFA1R_FFA10;																					//FIFO for filter (I use FIFO 1)
	CAN1->FS1R = CAN_FS1R_FSC10;																						//filter scale (doesn't matter, but preferably if you use 32bit format)
	CAN1->sFilterRegister[10].FR1 = 0;																			//Address filter (I don't use any for analyze all CAN packets)
	CAN1->sFilterRegister[10].FR2 = 0;
	CAN1->FMR &= (uint32_t)~CAN_FMR_FINIT;																	//End CAN filter initialization configuration
	
	/* BMW CAN-bus same as Toyota and configutarion setup will be same */
	
	CAN2->MCR = CAN_MCR_RESET;
	Timeout = 0xFFFF;
	while((CAN2->MSR&CAN_MCR_RESET) && Timeout != 0)
	{
		Timeout--;
	}
	if (Timeout == 0)
		CAN_init_Error = true;
	CAN2->MCR = CAN_MCR_INRQ | CAN_MCR_NART | CAN_MCR_AWUM;
	CAN2->IER = /*CAN_IER_FMPIE0 |*/ CAN_IER_FMPIE1 | CAN_IER_FOVIE1 | CAN_IER_TMEIE;
	CAN2->BTR = /*CAN_BTR_LBKM | */0x001c0003;
	CAN2->MCR &= ~((uint32_t)CAN_MCR_INRQ);
	Timeout = 0xFFFF;	
	while((CAN2->MSR&CAN_MSR_INAK) && Timeout != 0)
	{
		Timeout--;
	}
	if (Timeout == 0)
		CAN_init_Error = true;
	CAN2->FMR = CAN_FMR_FINIT;
	CAN2->FA1R = CAN_FA1R_FACT10;
	CAN2->FFA1R = CAN_FFA1R_FFA10;
	CAN2->FS1R = CAN_FS1R_FSC10;
	CAN2->sFilterRegister[10].FR1 = 0;
	CAN2->sFilterRegister[10].FR2 = 0;
	CAN2->FMR &= (uint32_t)~CAN_FMR_FINIT;
	
/* Transmit packets configuration */	
	
// Reverse gear packet for rear camera
	CAN_TxMessage[0].ID_ST = 0x3B0;										
	CAN_TxMessage[0].MessageLen = 0x2;
	CAN_TxMessage[0].DATA[0] = 0xFD;
	CAN_TxMessage[0].DATA[1] = 0xFF;	
	
// Hard Reset packet (must be removed)
	CAN_TxMessage[1].ID_ST = 0x6F1;
	//CAN_TxMessage[1].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[1].MessageLen = 0x4;
	CAN_TxMessage[1].DATA[0] = 0x5E;
	CAN_TxMessage[1].DATA[1] = 0x02;
	CAN_TxMessage[1].DATA[2] = 0x11;
	CAN_TxMessage[1].DATA[3] = 0x01;
	
// Gearbox position packet
	CAN_TxMessage[2].ID_ST = 0x3FD;
	//CAN_TxMessage[2].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[2].MessageLen = 0x5;
	CAN_TxMessage[2].DATA[1] = 0xFF;

// undefined packet (must be removed)
	CAN_TxMessage[3].ID_ST = 0x55E;
	//CAN_TxMessage[3].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[3].MessageLen = 0x2;
	CAN_TxMessage[3].DATA[0] = 0x5E;
	CAN_TxMessage[3].DATA[1] = 0xF1;
	
//Rough RPM packet 
	CAN_TxMessage[5].ID_ST = 0x0F3;
	//CAN_TxMessage[5].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[5].MessageLen = 0x8;
	CAN_TxMessage[5].DATA[1] = 0xFF;
	CAN_TxMessage[5].DATA[2] = 0x0;
	CAN_TxMessage[5].DATA[3] = 0xC0;
	CAN_TxMessage[5].DATA[4] = 0xF0;
	CAN_TxMessage[5].DATA[5] = 0x10;
	CAN_TxMessage[5].DATA[6] = 0xFF;
	CAN_TxMessage[5].DATA[7] = 0xFF;
	
// Throttle position and RPM
	CAN_TxMessage[6].ID_ST = 0x0A5;
	//CAN_TxMessage[6].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[6].MessageLen = 0x8;
	CAN_TxMessage[6].DATA[1] = 0xFF;
	CAN_TxMessage[6].DATA[2] = 0xEC;
	CAN_TxMessage[6].DATA[3] = 0x00;
	CAN_TxMessage[6].DATA[4] = 0x00;
	CAN_TxMessage[6].DATA[5] = 0x00;
	CAN_TxMessage[6].DATA[6] = 0x00;
	CAN_TxMessage[6].DATA[7] = 0xF1;
	
// Oil temperature packet
	CAN_TxMessage[7].ID_ST = 0x3F9;
	//CAN_TxMessage[7].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[7].MessageLen = 0x8;
	CAN_TxMessage[7].DATA[1] = 0xFF;
	CAN_TxMessage[7].DATA[2] = 0x82;
	CAN_TxMessage[7].DATA[3] = 0x4E;
	CAN_TxMessage[7].DATA[4] = 0x7E;
	CAN_TxMessage[7].DATA[5] = 0x00;
	CAN_TxMessage[7].DATA[6] = 0x05;
	CAN_TxMessage[7].DATA[7] = 0x89;

// Coolant temperature packet
	CAN_TxMessage[8].ID_ST = 0x2C4;
	//CAN_TxMessage[8].MessageDesc = CAN_TI0R_TXRQ;
	CAN_TxMessage[8].MessageLen = 0x8;
	CAN_TxMessage[8].DATA[1] = 0x3E;
	CAN_TxMessage[8].DATA[2] = 0x00;
	CAN_TxMessage[8].DATA[3] = 0x64;
	CAN_TxMessage[8].DATA[4] = 0x64;
	CAN_TxMessage[8].DATA[5] = 0x64;
	CAN_TxMessage[8].DATA[6] = 0x01;
	CAN_TxMessage[8].DATA[7] = 0xF1;	
	
// Speed packet (must be removed)
//	CAN_TxMessage[9].ID_ST = 0x1A1;
//	//CAN_TxMessage[9].MessageDesc = CAN_TI0R_TXRQ;
//	CAN_TxMessage[9].MessageLen = 0x5;
//	CAN_TxMessage[9].DATA[1] = 0xFF;
//	CAN_TxMessage[9].DATA[2] = 0;
//	CAN_TxMessage[9].DATA[3] = 0;
//	CAN_TxMessage[9].DATA[4] = 0x81;	
}

/* Definition CRC calulation function */
uint8_t CRC8(uint8_t Polynom, uint8_t InitValue, uint8_t XorValue, uint8_t *Buffer, uint8_t length)
{
	uint8_t CRCValue = InitValue;

	while (length--)
	{
		CRCValue ^= *Buffer++;
		for (uint8_t i = 0; i < 8; i++)
		{
			if (CRCValue & 0x80)
				CRCValue = (CRCValue << 1) ^ Polynom;
			else
				CRCValue <<= 1;
		}
	}
	return CRCValue ^ XorValue;
}

void CAN_Message_Send(uint8_t NumTxMessage)
{
	for (uint32_t i = 0; i <= 0xC; i += 4)
	{
		uint32_t u32var = *(uint32_t*)((uint32_t)&CAN_TxMessage[NumTxMessage] + (uint32_t)i);
		*(uint32_t*)((uint32_t)&CAN2->sTxMailBox[0] + (uint32_t)i) = u32var;
	}
	while (CAN2->MSR & CAN_MSR_STATUSMASK);
	CAN2->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;	
}

/* Definition CAN message transmission for BMW function */
void BMW_CAN_Message_Send(uint8_t NumTxMessage, uint8_t CRCPoly, uint8_t Length)
{
	CAN_TxMessage[NumTxMessage].DATA[1]++;
	if (CAN_TxMessage[NumTxMessage].DATA[1] == 0x0F)
		CAN_TxMessage[NumTxMessage].DATA[1] = 0;
	CAN_TxMessage[NumTxMessage].DATA[0] = CRC8(0x1D, 0xFF, CRCPoly, &CAN_TxMessage[NumTxMessage].DATA[1], Length);
	CAN_Message_Send(NumTxMessage);
}

void CAN1_RX1_IRQHandler(void)
{
	CAN1->RF1R = CAN_RF1R_RFOM1;
	if (CAN1->RF1R & CAN_RF1R_FOVR1)
		CAN1->RF1R = CAN_RF1R_FOVR1;
	for (uint32_t j = 0; j < 14; j++)
	{
		uint32_t u32var = *(uint32_t*)((uint32_t)&CAN1->sFIFOMailBox[1] + (j*4));
		*(uint32_t*)((uint32_t)&CAN_RxMessage2[0] + (j*4)) = u32var;
	}
	if (CAN_RxMessage2[0].ID_ST == 0x52C)
	{
		OILTEMP = CAN_RxMessage2[0].DATA[1] >> 1;
		CAN_TxMessage[7].DATA[5] = OILTEMP + 48;
		CAN_TxMessage[8].DATA[2] = OILTEMP + 26;
	}
	if (CAN_RxMessage2[0].ID_ST == 0x2C4)
	{
		RPM = (uint16_t)(((CAN_RxMessage2[0].DATA[0] << 8) + CAN_RxMessage2[0].DATA[1]) >> 2) * 3;
		RPMBMW = RPM / 2 * 3;
		CAN_TxMessage[5].DATA[2] = RPMBMW >> 8;		
		CAN_TxMessage[5].DATA[4] = (uint8_t)RPMBMW;
		CAN_TxMessage[6].DATA[5] = (uint8_t)RPM >> 6;
		CAN_TxMessage[6].DATA[6] = (uint8_t)RPM << 2;
	}
}

void CAN1_TX_IRQHandler(void)
{
	if (CAN1->TSR & CAN_TSR_TXOK0)
		CAN1->TSR |= CAN_TSR_RQCP0;
	else
		__NOP();
}

void CAN2_RX1_IRQHandler(void)
{
	CAN2->RF1R = CAN_RF1R_RFOM1;
	if (CAN2->RF1R & CAN_RF1R_FOVR1)
		CAN2->RF1R = CAN_RF1R_FOVR1;
/*
	uint32_t RxADDR = CAN2->sFIFOMailBox[1].RIR >> 21;
	if (RxADDR == 0x197)
		{
			for (uint32_t j = 0; j <= 0xC; j += 4)
			{
				uint32_t u32test = *(uint32_t*)((uint32_t)&CAN2->sFIFOMailBox[1] + j);
				*(uint32_t*)((uint32_t)&CAN_SwMessage + j) = u32test;
			}
		}
		for (uint8_t i = 0; i <= 4; i++)
		{
			if (RxADDR == CAN_RxMessage[i].ID_ST || CAN_RxMessage[i].ID_ST == 0)
			{
				index = i;
				break;
			}
		}
		for (uint32_t j = 0; j <= 0xC; j += 4)
		{
			uint32_t u32test = *(uint32_t*)((uint32_t)&CAN2->sFIFOMailBox[1] + j);
			*(uint32_t*)((uint32_t)&CAN_RxMessage[index] + j) = u32test;
		}
	*/
//This part must be reworked bc using hardreset and undefined messages
	if (HardReset == 0)
		CAN_HardReset();
	else
	{
		uint32_t RxADDR = CAN2->sFIFOMailBox[1].RIR >> 21;
		if (RxADDR == 0x65E)
		{
			CAN_TxMessage[1].ID_ST = 0x6F0;
			CAN_Message_Send(1);
		}
		if (RxADDR == 0x55E)
		{
			CAN_TxMessage[1].DATA[1] = 0x19;
			CAN_TxMessage[1].DATA[2] = 0x02;
			CAN_TxMessage[1].DATA[3] = 0x0C;
			CAN_Message_Send(3);	
		}
		if (RxADDR == 0x197)
		{
			for (uint32_t j = 0; j <= 0xC; j += 4)
			{
				uint32_t u32test = *(uint32_t*)((uint32_t)&CAN2->sFIFOMailBox[1] + j);
				*(uint32_t*)((uint32_t)&CAN_SwMessage + j) = u32test;
			}
		}
		for (uint8_t i = 0; i <= 4; i++)
		{
			if (RxADDR == CAN_RxMessage[i].ID_ST || CAN_RxMessage[i].ID_ST == 0)
			{
				index = i;
				break;
			}
		}
		for (uint32_t j = 0; j <= 0xC; j += 4)
		{
			uint32_t u32test = *(uint32_t*)((uint32_t)&CAN2->sFIFOMailBox[1] + j);
			*(uint32_t*)((uint32_t)&CAN_RxMessage[index] + j) = u32test;
		}
	}
}

void CAN2_TX_IRQHandler(void)
{
	if (CAN2->TSR & CAN_TSR_TXOK0)
		CAN2->TSR |= CAN_TSR_RQCP0;
	else
		__NOP();
}
