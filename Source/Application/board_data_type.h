/*
 * board_data_type.h
 *
 *  Created on: 15 ���. 2018 �.
 *      Author: real_bastard
 */

#ifndef BOARD_DATA_TYPE_H_
#define BOARD_DATA_TYPE_H_

#define NO_TRACK		0
#define TRACK_OK		1
#define TRACK_ERROR		2

typedef struct {
	uint32_t type;
	uint8_t data[100];
} udp_motor_pack_type;

typedef struct{
    int32_t HallTicks;//Счётчик тиков Холла
	uint16_t cur;
	uint16_t curLimit;
	uint16_t step;
	uint16_t pwmLimitFlag;//Флаг срабатывания pwm>=pwmMax
}tack_regs_t;

typedef struct {
    uint16_t scan_speed;
    uint16_t move_speed;
    uint16_t scan_step;
    uint16_t acs;
    float kSpeed[3];
    float kAcs[3];
    uint16_t cur_limit_mA[3];
} coef_track_dt;

typedef struct{
    uint16_t modbus_adr;
    uint16_t reg_adr;
    uint16_t size;
    char data[50];
}track_type;


typedef struct {
    uint32_t time;
    float Vin_min;
    float gX;
    float gY;
    float gZ;
    uint8_t trackState[3];		//состояние гусениц
    uint8_t BoardState;		//Не используется
    tack_regs_t reg[3];
    coef_track_dt coef;
    float ampl[24][2]; // [AIN0-23][0 - 300Hz 1 - 600Hz]
} track_data_type;

#endif /* BOARD_DATA_TYPE_H_ */
