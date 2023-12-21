/*
 * protocol.c
 *
 *  Created on: 31 мая 2018 г.
 *      Author: real_bastard
 */


#include <string.h>
#include "mbconfig.h"
#include "port.h"
#include "mb_m.h"
#include "mb.h"
#include "protocol.h"


#define REPEAT_TIME			100
#define MAX_REPEAT_COUNT	5

#define M_REG_HOLDING_START				0x00
#define M_REG_HOLDING_NREGS				40

/*-----------------------Master mode use these variables----------------------*/
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0

//Master mode:InputRegister variables
//USHORT   usMRegInStart                              = M_REG_INPUT_START;
//USHORT   usMRegInBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_INPUT_NREGS];
//Master mode:HoldingRegister variables
USHORT usMRegHoldStart = M_REG_HOLDING_START;
USHORT usMRegHoldBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];
#endif

//extern robot_data_type data;
extern track_type track_data;
static protocol_state_t p_state;
static uint32_t delay_time;
static uint32_t repeat_count;
static uint16_t dev_adr = FIRST_MODBUS_ADR;
extern void xMBPortSerial_IRQHandler(void);
static int32_t encoder[3];
static uint32_t trackState[3]={0,0,0};

static uint16_t regAdr;
static uint16_t regSize;

static int rx_flag[3]={0,0,0};

tack_regs_t regs[3];

void protocol_init(void) {
	eMBMasterInit(MB_RTU, 0,				//Not Used
			115200,			// Baud
			MB_PAR_NONE);  // Par
	Delay_ms(1000);
	eMBMasterInit(MB_RTU, 0,				//Not Used
			115200,			// Baud
			MB_PAR_NONE);  // Par
	eMBErrorCode eStatus;   // modbus status
	//     Enable the Modbus Protocol Stack.
	eStatus = eMBMasterEnable();
	while (1) {
		if (eMBMasterPoll() == MB_ENOERR)
			break;
	}
	protocol_start();

}

void protocol_start(void) {
	p_state = pm_NA;
	p_state = pm_ready;
	delay_time = 0;
	repeat_count = 0;
}

void protocol_write_buf(uint8_t* data,int size){
	if (size==sizeof(track_data))
		memcpy((char*) &track_data, &data[4], sizeof(track_data));
}



/*int32_t protocol_getEnc(uint32_t num){
	return regs[num].enc;
}

uint32_t protocol_getSensState(uint32_t num){
	return regs[num].mode;
}*/

tack_regs_t protocol_regs(uint32_t num){
	return regs[num];
}

int protocol_getRxData(int track){
	if (rx_flag[track]==1){
		rx_flag[track]=0;
		return 1;
	}
	return 0;
}

bool protocol_write(void) {
	switch (p_state) {
	case pm_write:  default:
		return false;
	case pm_ready: case pm_read:
		p_state = pm_write;
		regAdr=track_data.reg_adr;
		dev_adr=track_data.modbus_adr;
		regSize=track_data.size;
		return true;
	case pm_read_cmlt:
		p_state = pm_write_after_read;
		regAdr=track_data.reg_adr;
		dev_adr=track_data.modbus_adr;
		regSize=track_data.size;
		return true;
	case pm_stopped:
		p_state = pm_write;
		regAdr = track_data.reg_adr;
		dev_adr = track_data.modbus_adr;
		regSize = track_data.size;
		return true;
	}


}

bool protocol_read(void) {
	if (p_state != pm_ready)
		return false;
	p_state = pm_read;
	return true;
}

protocol_state_t protocol_process(void) {
	USHORT* data_ptr;
	eMBMasterPoll();
	switch (p_state) {
	case pm_read:
		dev_adr = FIRST_MODBUS_ADR;
		eMBMasterReqReadHoldingRegister(dev_adr, 20, 4, 100);
		delay_time = getTime_ms() + REPEAT_TIME;
		repeat_count = 0;
		p_state = pm_read_cmlt;
		break;
	case pm_read_cmlt:
		if (eMBMasterWaitRequestFinish() == MB_MRE_NO_ERR) {
			regs[dev_adr-FIRST_MODBUS_ADR].cur=usMRegHoldBuf[dev_adr-1][20];
			regs[dev_adr-FIRST_MODBUS_ADR].curLimit=usMRegHoldBuf[dev_adr-1][21];
			regs[dev_adr-FIRST_MODBUS_ADR].step=usMRegHoldBuf[dev_adr-1][22];
			regs[dev_adr-FIRST_MODBUS_ADR].res=usMRegHoldBuf[dev_adr-1][23];
			rx_flag[dev_adr-FIRST_MODBUS_ADR]=1;
			dev_adr++;
			if (dev_adr > LAST_MODBUS_ADR) {
				p_state = pm_ready;
			} else {
				eMBMasterReqReadHoldingRegister(dev_adr, 20, 4, 100);
				delay_time = getTime_ms() + REPEAT_TIME;
				repeat_count = 0;
			}
		} else if (eMBMasterWaitRequestFinish() != MB_MRE_MASTER_BUSY) {
			if (getTime_ms() >= delay_time) {
				if (repeat_count < MAX_REPEAT_COUNT) {
					eMBMasterReqReadHoldingRegister(dev_adr, 20, 4, 100);
					repeat_count++;
					delay_time = getTime_ms() + REPEAT_TIME;
				} else {
					encoder[dev_adr-FIRST_MODBUS_ADR] = 0;
					trackState[dev_adr-FIRST_MODBUS_ADR] = 0;
					dev_adr++;
					if (dev_adr > LAST_MODBUS_ADR) {
						p_state = pm_ready;
					} else {
						eMBMasterReqReadHoldingRegister(dev_adr, 20, 4, 100);
						delay_time = getTime_ms() + REPEAT_TIME;
						repeat_count = 0;
					}
				}
			}
		}
		break;
	case pm_write_after_read:
		if (eMBMasterWaitRequestFinish() == MB_MRE_NO_ERR) {
			p_state = pm_write;
		}else if (eMBMasterWaitRequestFinish() != MB_MRE_MASTER_BUSY) {
			if (getTime_ms() >= delay_time) {
				p_state = pm_write;
			}
		}
		break;
	case pm_error:
		break;
	case pm_write:
		eMBMasterReqWriteMultipleHoldingRegister(dev_adr,
				regAdr, regSize, (USHORT*) track_data.data,
				10);
		delay_time = getTime_ms() + REPEAT_TIME;
		p_state = pm_write_cmplt;

		break;
	case pm_ready:
		break;
	case pm_write_cmplt:
		if (eMBMasterWaitRequestFinish() == MB_MRE_NO_ERR) {
			p_state = pm_ready;
		}else if (eMBMasterWaitRequestFinish() != MB_MRE_MASTER_BUSY) {
			if (getTime_ms() >= delay_time) {
				trackState[0] = 0;
				trackState[1] = 0;
				trackState[2] = 0;
				p_state = pm_stopped;
			}
		}
		break;
	case pm_stopped:
		break;
	default:
		p_state = pm_ready;
		break;
	}
	return p_state;
}

protocol_state_t protocol_getState(void) {
	return p_state;
}

void protocol_IRQHandler(void) {
	xMBPortSerial_IRQHandler();
}

eMBErrorCode eMBMasterRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNCoils, eMBRegisterMode eMode) {
	return MB_ENOERR;
}

eMBErrorCode eMBMasterRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNDiscrete) {
	return MB_ENOERR;
}

eMBErrorCode eMBMasterRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs) {
	return MB_ENOERR;
}

/**
 * Modbus master holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBMasterRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	USHORT iRegIndex;
	USHORT * pusRegHoldingBuf;
	USHORT REG_HOLDING_START;
	USHORT REG_HOLDING_NREGS;
	USHORT usRegHoldStart;

	pusRegHoldingBuf = usMRegHoldBuf[ucMBMasterGetDestAddress() - 1];
	REG_HOLDING_START = M_REG_HOLDING_START;
	REG_HOLDING_NREGS = M_REG_HOLDING_NREGS;
	usRegHoldStart = usMRegHoldStart;

	volatile static int at;
	volatile static int ad;
	volatile static int n;
	volatile static int m;
	at=ucMBMasterGetDestAddress();
	ad=usAddress;
	 n=usNRegs;
	 m=eMode;
	/* if mode is read, the master will write the received date to buffer. */
	eMode = MB_REG_WRITE;

	/* it already plus one in modbus function method. */
	usAddress--;

	if ((usAddress >= REG_HOLDING_START)
			&& (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = usAddress - usRegHoldStart;
		switch (eMode) {
		/* read current register values from the protocol stack. */
		case MB_REG_READ:
			while (usNRegs > 0) {
				*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
				*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
				iRegIndex++;
				usNRegs--;
			}
			break;
			/* write current register values with new values from the protocol stack. */
		case MB_REG_WRITE:
			while (usNRegs > 0) {

				pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++;
				pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++ << 8;

				iRegIndex++;
				usNRegs--;
			}
			break;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

