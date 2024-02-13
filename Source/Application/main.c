/**
 ******************************************************************************
 * @file    USART/USART_Printf/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    18-April-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "string.h"
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "string.h"
#include "Trace.h"
#include "motor.h"
#include "lsm330.h"
#include "board_data_type.h"
#include "protocol.h"
#include "stm32f4x7_eth.h"
#include "track.h"
#include "../GD32F4xx_Firmware_Library_V3.1.0/systick.h"
#include "../GD32F4xx_Firmware_Library_V3.1.0/gd32f4xx_libopt.h"
#include "mux.h"
#include "adc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  1

typedef struct {
	uint32_t pac_type;
	int8_t data[1024 - 4];	//������ � ����������
} pack_type;


__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */


track_data_type data;
uint32_t rxByteCount = 0;
uint32_t udpPacketNum = 0;
pack_type udp_Pack;
struct udp_pcb *upcb;
volatile uint32_t wait_cmd_time = 0;
volatile uint32_t modbus_write = 0;

//COEF
coef_track_dt coef;
bool updateCoefFlag = false;

//LwIP
bool LwIp_initFlag = false;

//SCAN

typedef enum {
	cmd_na, cmd_stop, cmd_scanFwd, cmd_scanRev, cmd_moveFwd, cmd_moveRev
} cmd_type;

cmd_type active_cmd = cmd_na;



static uint16_t last_step[3];

static uint16_t track_move_speed;
static uint16_t track_scan_speed;

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		struct ip_addr *addr, u16_t port);
void udp_sendBuf(char* udpTxBuf);

#define COM_PORT                        USART0
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
    usart_data_transmit(COM_PORT, (uint8_t)ch);
    while(RESET == usart_flag_get(COM_PORT, USART_FLAG_TBE));
    return ch;
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(COM_PORT, (uint8_t)ch);
    while(RESET == usart_flag_get(COM_PORT, USART_FLAG_TBE));
    return ch;
}



/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	uint32_t index = 0;
	struct ip_addr DestIPaddr;
	err_t err;
	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32f2xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f4xx.c file
	 */
    SystemCoreClockUpdate();
    systick_config();
	__enable_irq();
	Delay_ms(100);
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	//lsm330_setup(LSM330_GYR_FS_250DPS, LSM330_ACC_G_2G);
	Delay_ms(100);
	Delay_ms(10);
	coef.scan_speed = 300;
	coef.move_speed = 300;
	coef.scan_step = 300;
	coef.kSpeed[0] = 1;
	coef.kSpeed[1] = 1;
	coef.kSpeed[2] = 1;

	coef.acs = 150;
	coef.kAcs[0] = 1;
	coef.kAcs[1] = 1;
	coef.kAcs[2] = 1;

	protocol_init();
    //vMBMasterPortSerialEnable( 0, 1);   
	
	float speedL;
	float speedR;
	float speed;
	float rotation;
	float fastRotation;
	float direction;

    spi_mux_config();
	Delay_ms(10);
	mux_reset(0);
while(1){
	mux_set_sync(1);
	spi_mux_send(0x55);
	mux_set_sync(0);
	mux_set_sync(1);
	spi_mux_send(0x55);
	mux_set_sync(0);
	

	Delay_ms(100);
}
	//Delay_ms(1);
	//mux_set_sync(1);
	//spi_mux_send(0x02);
	//mux_set_sync(0);

	//while(1){;}
	/* configure ethernet (GPIOs, clocks, MAC, DMA) */
	//ETH_BSP_Config();

	enet_system_setup();
	/* Initilaize the LwIP stack */
	LwIP_Init();
	LwIp_initFlag = true;
	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb != NULL) {
		/*assign destination IP address */
		IP4_ADDR(&DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2,
				DEST_IP_ADDR3);
		err = udp_bind(upcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
		if (err == ERR_OK) {
			udp_recv(upcb, udp_receive_callback, NULL);
		}
		/* configure destination IP address and port */
		err = udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);

		if (err == ERR_OK) {
			/* Set a receive callback for the upcb */
			udp_recv(upcb, udp_receive_callback, NULL);
		} else {
		}
	} else {
	}

	Delay_ms(10);
	lsm330_setup(LSM330_GYR_FS_250DPS, LSM330_ACC_G_2G);
	uint32_t readTime = 0;
	uint32_t motorTime = 0;

	udpPacketNum = 0;

	/* Infinite loop */

	modbus_write = 0;

	uint32_t last_data_count = 0;

	track_update_speed(coef.move_speed, coef.kSpeed);
	track_update_scan_speed(coef.scan_speed, coef.kSpeed);
	Delay_ms(2);
	track_update_step(coef.scan_step, coef.kSpeed);
	track_move_speed = coef.move_speed;
	track_scan_speed = coef.scan_speed;

	//track_scan_fwd();
	uint8_t* dataPtr=(uint8_t*) &data;

	while (1) {


		protocol_process();


		switch (active_cmd) {
		case cmd_na:
		default:
			break;
		case cmd_stop:
			//TEST PIN
			GPIO_ResetBits(GPIOF_stm, GPIO_Pin_4);
			track_stop();
			active_cmd = cmd_na;
			break;
		case cmd_scanFwd:
			last_step[0] = data.reg[0].step;
			last_step[1] = data.reg[1].step;
			last_step[2] = data.reg[2].step;
			if (track_scan_speed != coef.scan_speed) {
				track_scan_speed = coef.scan_speed;
				track_update_scan_speed(coef.scan_speed, coef.kSpeed);
				Delay_ms(2);
			}
			track_scan_fwd();
			active_cmd = cmd_na;
			break;
		case cmd_scanRev:
			last_step[0] = data.reg[0].step;
			last_step[1] = data.reg[1].step;
			last_step[2] = data.reg[2].step;
			if (track_scan_speed != coef.scan_speed) {
				track_scan_speed = coef.scan_speed;
				track_update_scan_speed(track_scan_speed, coef.kSpeed);
				Delay_ms(2);
			}
			track_scan_rev();
			active_cmd = cmd_na;
			break;
		case cmd_moveFwd:
			//TEST PIN
			GPIO_SetBits(GPIOF_stm, GPIO_Pin_4);
			if (track_move_speed != coef.move_speed) {
				track_move_speed = coef.move_speed;
				track_update_speed(track_move_speed, coef.kSpeed);
			}
			track_fwd();
			active_cmd = cmd_na;
			break;
		case cmd_moveRev:
			if (track_move_speed != coef.move_speed) {
				track_move_speed = coef.move_speed;
				track_update_speed(track_move_speed, coef.kSpeed);
			}
			track_rev();
			active_cmd = cmd_na;
			break;
		}
		//Это обработка принятой команды на прямую запись в модбас
		if (modbus_write != 0) {
			if (protocol_write())
				modbus_write = 0;
		}
		//Это обработка принятой команды на смену коэффициентов в гусенице
		if (updateCoefFlag) {
			updateCoefFlag = false;
			track_move_speed = coef.move_speed;
			track_scan_speed = coef.scan_speed;
			track_update_speed(coef.move_speed, coef.kSpeed);
			track_update_scan_speed(coef.scan_speed, coef.kSpeed);
			track_update_step(coef.scan_step, coef.kSpeed);
			track_update_acs(coef.acs, coef.kAcs);
			track_update_Ilimit(coef.cur_limit_mA);
		}

		//потеря связи с ПК
		if (wait_cmd_time > 2000) {
			wait_cmd_time = 0;
			track_stop();
			//udp_printf("track_stop");
		}
		//периодическая посылка на ПК
		if (getTime_ms() > readTime) {
			data.time++;
			getAccelValues();
			protocol_read();
			data.gX = scaledAccelX_;
			data.gY = scaledAccelY_;
			data.gZ = scaledAccelZ_;
			for (int i = 0; i < 3; i++) {
				if (protocol_getRxData(i) == 1) {
					data.reg[i] = protocol_regs(i);
				}else{
					//здесь можно поставить флаг отсутствия связи с гусеницей
				}

			}
			udp_Pack.pac_type = 0;
			for (int i = 0; i < sizeof(data); i++) {
				udp_Pack.data[i] = dataPtr[i];
			}
			data.coef = coef;
			udp_sendBuf((uint8_t*) &udp_Pack);
			readTime = getTime_ms() + 100;
		}
	}
}



int udp_printf(const char* format, ...) {
	int ret;
	va_list ap;
	struct pbuf *p;
	char* sPtr;
	static char udpTxBuf[1024];
	va_start(ap, format);

	// TODO: rewrite it to no longer use newlib, it is way too heavy

	// Print to the local buffer
	ret = vsnprintf(udpTxBuf, sizeof(udpTxBuf), format, ap);
	if (ret > 0) {
		/* allocate pbuf from pool*/
		p = pbuf_alloc(PBUF_TRANSPORT, ret, PBUF_POOL);
		sPtr = (char*) p->payload;

		for (int index = 0; index < ret; index++) {
			sPtr[index] = udpTxBuf[index];
		}
		/* send udp data */
		if (udp_send(upcb, p) == ERR_OK) {

		} else {

		}
		pbuf_free(p);
	}

	va_end(ap);
	return ret;
}

void udp_sendBuf(char* udpTxBuf) {
	struct pbuf *p;
	char* sPtr;

	/* allocate pbuf from pool*/
	p = pbuf_alloc(PBUF_TRANSPORT, 1024, PBUF_POOL);
	sPtr = (char*) p->payload;

	for (int index = 0; index < 1024; index++) {
		sPtr[index] = udpTxBuf[index];
	}
	/* send udp data */
	if (udp_send(upcb, p) == ERR_OK) {

	} else {

	}
	pbuf_free(p);
}

/**
 * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
 * @param arg user supplied argument (udp_pcb.recv_arg)
 * @param pcb the udp_pcb which received data
 * @param p the packet buffer that was received
 * @param addr the remote IP address from which the packet was received
 * @param port the remote port from which the packet was received
 * @retval None
 */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		struct ip_addr *addr, u16_t port) {
	udp_motor_pack_type* motor_pack;
	uint8_t rxData[1200];
	memset(rxData,0,1200);
	wait_cmd_time = 0;
	rxByteCount = p->tot_len;
	MEMCPY((char* )rxData, p->payload, rxByteCount);
	motor_pack = (udp_motor_pack_type*) rxData;
	if (motor_pack->type == 0x01020304) {
		wait_cmd_time = 0;
		rxByteCount = p->tot_len;
		protocol_write_buf(&rxData[4], rxByteCount - 4);
		modbus_write = 10;
		pbuf_free(p);
	} else if (motor_pack->type == 0x10000001) {
		memcpy((char*) &coef, &rxData[4], sizeof(coef));
		updateCoefFlag = true;
	} else if (motor_pack->type == 0x20000001) {
		active_cmd=cmd_scanFwd;
	} else if (motor_pack->type == 0x20000000) {
		active_cmd=cmd_stop;
	} else if (motor_pack->type == 0x20000002) {
		active_cmd=cmd_moveFwd;
	} else if (motor_pack->type == 0x20000003) {
		active_cmd=cmd_moveRev;
	} else if (motor_pack->type == 0x20000006) {
		active_cmd=cmd_scanRev;
	}
	/* Free receive pbuf */
	pbuf_free(p);
}

/**
 * @brief  Inserts a delay time.
 * @param  nCount: number of 1ms periods to wait for.
 * @retval None
 */
void Delay_ms(uint32_t nCount) {
	volatile uint32_t timingdelay = getTime_ms() + nCount;
	/* wait until the desired delay finish */
	while (timingdelay > getTime_ms()) {
	}
}

/**
 * @brief  Updates the system local time
 * @param  None
 * @retval None
 */
void Time_Update(void) {
	LocalTime += SYSTEMTICK_PERIOD_MS;
	wait_cmd_time++;
	if (!LwIp_initFlag)
		return;
	/* handle periodic timers for LwIP */
	LwIP_Periodic_Handle(LocalTime);
	// check if any packet received
	if (ETH_CheckFrameReceived()) {
		// process received ethernet packet
		LwIP_Pkt_Handle();
	}
}

uint32_t getTime_ms(void) {
	return LocalTime;
}

void __errno(void) //dummy
{
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
