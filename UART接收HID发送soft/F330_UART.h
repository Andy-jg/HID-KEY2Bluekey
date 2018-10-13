
#ifndef F330_UART_H

#define F330_UART_H

#define SysPow_ON	1
#define SysPow_OFF	0
#define SysBL_ON	0
#define SysBL_OFF	1

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSTEMCLOCK      6125000           // SYSCLK frequency in Hz
#define BAUDRATE         57600                // Baud rate of UART in bps

#define TIMER_PRESCALER            48  // Based on Timer CKCON settings

// There are SYSTEMCLOCK/TIMER_PRESCALER timer ticks per second, so
// SYSCLK/TIMER_PRESCALER/1000 timer ticks per millisecond.
#define TIMER_TICKS_PER_MS  SYSTEMCLOCK/TIMER_PRESCALER/1000


#define UART_BUFFERSIZE 48

// Note: TIMER_TICKS_PER_MS should not exceed 255 (0xFF) for the 8-bit timer

#define AUX1     TIMER_TICKS_PER_MS
#define AUX2     -AUX1

#define TIMER0_RELOAD_HIGH       AUX2  // Reload value for Timer0 high byte

#define HID_FnDown	0x00
#define HID_FnUp	0x01

//key
#define HID_LCtrl 	0x14
#define HID_LShift 	0x12
#define HID_LAlt		0x10
#define HID_LGui		0x07
//#define RCtrl	0x??
#define HID_RShift 	0x59

#define HID_RAlt		0x13
//#define RGui	0x??
#define HID_Fn		0x02
//fn_key,用于测试。实际使用时查表

#define HID_Home	0x5E
#define HID_End		0x2F
#define HID_PgUp	0x28
#define HID_PgDown	0x60
#define HID_Esc		0x0E
#define HID_F1		0x16
#define HID_F3		0x26
#define TRUE			(1)
#define FALSE		(0)
#endif

