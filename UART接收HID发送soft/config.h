#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "mytype.h"

//#define Voltage_REF 3300UL	  //ADC参考电压，即MCU电源电压
//#define MODE1T

#define on	1
#define off	0

#define t50ms		1
#define t100ms		2
#define t150ms		3
#define t200ms		4
#define t300ms		6
#define t500ms		15
#define t1s			20
#define t2s			40
#define t3s			60
#define t5s			100
#define t10s		200

sbit	vcc_ctrl	=	P0^2;
sbit	key_int0	=	P0^3;
sbit	zb_led2		=	P0^6;
sbit	bl_conn_ctrl=	P1^3;
sbit	bl_state	=	P1^4;
sbit	bl_vcc_ctrl	=	P1^6;
sbit	zb_adc0		=	P1^7;


void Init_Device();
void led_rgb(uint8 t);
void InitIO(void);
void InitTimer0(void);
void InitTimer1(void);
void InitUART2(void);
void delay_us(uint8 us) ;
void delay_ms(uint16 ms) ;
void delay(uint16 n) ;
void delayNOP();

#endif