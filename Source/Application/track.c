/*
 * track.c
 *
 *  Created on: 23 ���. 2020 �.
 *      Author: A-User
 */


#include "protocol.h"
#include "track.h"

#define MOVE_SPEED_REG_ADR		0x0C
#define SCAN_SPEED_REG_ADR		0x0D
#define SCAN_STEP_REG_ADR		0x10
#define ACS_RISE_REG_ADR		0x05
#define I_LIMIT_REG_ADR			0x03
#define CONTROL_REG_ADR			0x02
#define CNT_DRIVERS             1U
#define broadcast_messages      0U

#define	MOVE_FWD	0x02
#define	MOVE_REV	0x01
#define	STEP_FWD	0x04
#define	STEP_REV	0x03
extern __IO uint32_t LocalTime;
track_type track_data;


void track_process(void){

}

void track_update_speed(uint16_t speed, float* kSpeed) {
	uint32_t delay;
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint16_t cSpeed;
	track_data.reg_adr = MOVE_SPEED_REG_ADR;
	track_data.size = 1;

	for (int j = 0; j < CNT_DRIVERS; j++) {
		track_data.modbus_adr = FIRST_MODBUS_ADR + j;
		cSpeed=(float) speed * kSpeed[j] + 0.5;
		siPtr[0] = cSpeed;
		for (int i = 0; i < 3; i++) {
			protocol_write();
			delay = LocalTime + 100;
			while (1) {
				if (LocalTime > delay) {
					break;
				}
				if (protocol_process() == pm_ready)
					break;
				if (protocol_process() == pm_stopped)
					break;
			}
		}
	}
}

void track_update_scan_speed(uint16_t speed,float* kSpeed) {
	uint32_t delay;
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint16_t cSpeed;
	track_data.reg_adr = SCAN_SPEED_REG_ADR;
	track_data.size = 1;

	for (int j = 0; j < CNT_DRIVERS; j++) {
		track_data.modbus_adr = FIRST_MODBUS_ADR + j;
		cSpeed=(float) speed * kSpeed[j] + 0.5;
		siPtr[0] = cSpeed;
		for (int i = 0; i < 3; i++) {
			protocol_write();
			delay = LocalTime + 100;
			while (1) {
				if (LocalTime > delay) {
					break;
				}
				if (protocol_process() == pm_ready)
					break;
				if (protocol_process() == pm_stopped)
					break;
			}
		}
	}
}


void track_update_step(uint16_t step, float* kSpeed) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	track_data.reg_adr = SCAN_STEP_REG_ADR;
	track_data.size = 1;

	for (int j = 0; j < CNT_DRIVERS; j++) {
		track_data.modbus_adr = FIRST_MODBUS_ADR + j;
		siPtr[0] = (float) step * kSpeed[j] + 0.5;
		for (int i = 0; i < 3; i++) {
			protocol_write();
			delay = LocalTime + 100;
			while (1) {
				if (LocalTime > delay) {
					break;
				}
				if (protocol_process() == pm_ready)
					break;
				if (protocol_process() == pm_stopped)
					break;
			}
		}
	}
}


void track_update_acs(uint16_t acs, float* kAcs) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	track_data.reg_adr = ACS_RISE_REG_ADR;
	track_data.size = 1;

	for (int j = 0; j < CNT_DRIVERS; j++) {
		track_data.modbus_adr = FIRST_MODBUS_ADR + j;
		siPtr[0] = (float) acs * kAcs[j] + 0.5;
		for (int i = 0; i < 3; i++) {
			protocol_write();
			delay = LocalTime + 100;
			while (1) {
				if (LocalTime > delay) {
					break;
				}
				if (protocol_process() == pm_ready)
					break;
				if (protocol_process() == pm_stopped)
					break;
			}
		}
	}
}



void track_update_Ilimit(uint16_t* Ilimit) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	track_data.reg_adr = I_LIMIT_REG_ADR;
	track_data.size = 1;

	for (int j = 0; j < CNT_DRIVERS; j++) {
		track_data.modbus_adr = FIRST_MODBUS_ADR + j;
		siPtr[0] = Ilimit[j];
		for (int i = 0; i < 3; i++) {
			protocol_write();
			delay = LocalTime + 100;
			while (1) {
				if (LocalTime > delay) {
					break;
				}
				if (protocol_process() == pm_ready)
					break;
				if (protocol_process() == pm_stopped)
					break;
			}
		}
	}
}

void track_scan_fwd(void) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	siPtr[0] = STEP_FWD;

	track_data.modbus_adr = broadcast_messages;
	track_data.reg_adr = CONTROL_REG_ADR;
	track_data.size = 1;

	for (int i = 0; i < 3; i++) {
		protocol_write();
		delay = LocalTime + 100;
		while (1) {
			if (LocalTime > delay) {
				break;
			}
			if (protocol_process() == pm_ready)
				break;
			if (protocol_process() == pm_stopped)
				break;
		}
	}
}



void track_scan_rev(void) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	siPtr[0] = STEP_REV;

	track_data.modbus_adr = broadcast_messages;
	track_data.reg_adr = CONTROL_REG_ADR;
	track_data.size = 1;

	for (int i = 0; i < 3; i++) {
		protocol_write();
		delay = LocalTime + 100;
		while (1) {
			if (LocalTime > delay) {
				break;
			}
			if (protocol_process() == pm_ready)
				break;
			if (protocol_process() == pm_stopped)
				break;
		}
	}
}

void track_stop(void) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	siPtr[0] = 0;
	track_data.modbus_adr = broadcast_messages;
	track_data.reg_adr = CONTROL_REG_ADR;
	track_data.size = 1;

	for (int i = 0; i < 3; i++) {
		protocol_write();
		delay = LocalTime + 500;
		while (1) {
			if (LocalTime > delay) {
				break;
			}
			if (protocol_process() == pm_ready)
				break;
			if (protocol_process() == pm_stopped)
				break;
		}
	}
}

void track_fwd(void) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	siPtr[0] = MOVE_FWD;

	track_data.modbus_adr = broadcast_messages;
	track_data.reg_adr = CONTROL_REG_ADR;
	track_data.size = 1;
	for (int i = 0; i < 3; i++) {
		protocol_write();
		delay = LocalTime + 100;
		while (1) {
			if (LocalTime > delay) {
				break;
			}
			if (protocol_process() == pm_ready)
				break;
			if (protocol_process() == pm_stopped)
				break;
		}
	}
}

void track_rev(void) {
	uint16_t* siPtr = (uint16_t*) track_data.data;
	uint32_t delay;
	siPtr[0] = MOVE_REV;

	track_data.modbus_adr = broadcast_messages;
	track_data.reg_adr = CONTROL_REG_ADR;
	track_data.size = 1;
	for (int i = 0; i < 3; i++) {
		protocol_write();
		delay = LocalTime + 100;
		while (1) {
			if (LocalTime > delay) {
				break;
			}
			if (protocol_process() == pm_ready)
				break;
			if (protocol_process() == pm_stopped)
				break;
		}
	}
}
