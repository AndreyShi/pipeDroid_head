/*
 * protocol.h
 *
 *  Created on: 31 мая 2018 г.
 *      Author: real_bastard
 */

#ifndef SOURCE_APPLICATION_PROTOCOL_H_
#define SOURCE_APPLICATION_PROTOCOL_H_
#include "stdbool.h"
#include "main.h"
#include "board_data_type.h"

typedef enum {
	pm_NA,
	pm_find,
	pm_read,
	pm_read_cmlt,
	pm_write_after_read,
	pm_write,
	pm_write_cmplt,
	pm_ready,
	pm_error,
	pm_stopped
} protocol_state_t;



void protocol_init(void);
void protocol_start(void);
void protocol_write_buf(uint8_t* data,int size);
bool protocol_write(void);
bool protocol_read(void);
protocol_state_t protocol_process(void);
void protocol_IRQHandler(void);
//int32_t protocol_getEnc(uint32_t num);
//uint32_t protocol_getSensState(uint32_t num);
tack_regs_t protocol_regs(uint32_t num);
int protocol_getRxData(int track);
int getVmin(void);


#endif /* SOURCE_APPLICATION_PROTOCOL_H_ */
