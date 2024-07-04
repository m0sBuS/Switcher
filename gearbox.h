#include <stm32f407xx.h>
#include <stdbool.h>
//#include "CAN.h"

#define P_POS				GPIO_IDR_ID9
#define R_POS				GPIO_IDR_ID10
#define N_POS				GPIO_IDR_ID11
#define D_POS				GPIO_IDR_ID12
#define S_POS				GPIO_IDR_ID13
#define BREAK_POS		GPIO_IDR_ID8

#define PARKING_PWM	(uint32_t)2840
#define SWITCH_PWM	(uint32_t)1500
#define BRAKE_DELAY	(uint16_t)100				//Count milliseconds dynamic braking delay
#define BRAKE_PWM		(uint32_t)900				//Reverse PWM brake ratio

enum GB_Pos_enum{
	UNDEFINED = 0,
	PARKING = P_POS,
	REVERSE = R_POS,
	NEUTRAL = N_POS,
	DRIVE = D_POS,
	MANUAL = S_POS
};

void GB_Init(void);
void GB_CANRequest(void);
void GB_SwPos(uint16_t PWM, uint8_t DIR, uint8_t Brake);
void GB_Position(void);
void CAN_GBPos(enum GB_Pos_enum GB_Pos);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

static enum GB_Pos_enum GB_Status = UNDEFINED;
static enum GB_Pos_enum GB_Request = PARKING;

static uint8_t Timeout = 0;
static uint16_t Braking = 0;
static bool CNT_Reset = 0;
static bool OneSwitch = 0;

static uint32_t testu32 =0;
