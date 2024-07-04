#include <stm32f407xx.h>
#include <stdbool.h>

#define CAN_MSR_STATUSMASK	CAN_MSR_TXM & CAN_MSR_RXM & CAN_MSR_SAMP & CAN_MSR_RX

void CAN_HardReset(void);
void CAN_Init(void);
uint8_t CRC8(uint8_t Polynom, uint8_t InitValue, uint8_t XorValue, uint8_t *Buffer, uint8_t length);
void CAN_Message_Send(uint8_t NumTxMessage);
void BMW_CAN_Message_Send(uint8_t NumTxMessage, uint8_t CRCPoly, uint8_t Length);

void CAN1_RX1_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
void CAN2_TX_IRQHandler(void);

typedef struct{ 
	unsigned char 	MessageDesc 	: 3;
	unsigned int 		ID_EX 				: 18;
	unsigned short	ID_ST 				: 11;
	unsigned char		MessageLen		:	4;
	unsigned char		Res						:	4;
	unsigned char		FilterMatch		:	8;
	unsigned short	Timestamp;
	unsigned char		DATA[8];
}CAN_MessageStruct;

static CAN_MessageStruct CAN_SwMessage;
static CAN_MessageStruct CAN_RxMessage[5] = {0};
static CAN_MessageStruct CAN_RxMessage2[15] = {0};
static CAN_MessageStruct CAN_TxMessage[15] = {0};

static uint16_t SPEEDBMW = 0;

static uint8_t OILTEMP = 0, FREESETEMP = 0;
static uint16_t RPM = 0, SPEED = 0, RPMBMW = 0;
static uint8_t index, index2;
static bool HardReset = 0, CAN_init_Error = 0, startPump = 0, CAN1_TxReq;
