/*Program writen by Maxim Dorokhin https://github.com/m0sBuS
	
	Header-file for custom merging curcuit board BMW F02 CAN-bus and TOYOTA 3UZ-FE motor.
	This includes stable work on KEIL µVision V5.38.0.0
	
*/

#include <stm32f407xx.h>
#include "stdbool.h"
//#include "core.h"
//#include "gearbox.h"
#include "CAN.c"
#include "gearbox.c"

void RCCPLL_init(void);
void MS_Timer_Init(void);
void TIM1_BRK_TIM9_IRQHandler(void);

extern enum GB_Pos_enum GB_Status;
extern enum GB_Pos_enum GB_Request;

//extern CAN_MessageStruct CAN_SwMessage;
extern CAN_MessageStruct CAN_RxMessage[5];
extern CAN_MessageStruct CAN_RxMessage2[15];
extern CAN_MessageStruct CAN_TxMessage[15];

extern uint16_t Braking;
static uint8_t TestCnt = 0; 
static uint8_t PacketCNT = 0, PrevPacket = 0;

static uint8_t CANGearPosition100ms = 0;
static uint16_t CANReversePacket2s = 5;

extern bool HardReset;
extern uint8_t TEMP;
extern uint16_t RPM, SPEED, RPMBMW;
