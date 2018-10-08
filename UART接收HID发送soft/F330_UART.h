

#define vcc_on	1
#define vcc_off	0
#define bl_on	0
#define bl_off	1

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSTEMCLOCK      6125000           // SYSCLK frequency in Hz
#define BAUDRATE         57600                // Baud rate of UART in bps

#define TIMER_PRESCALER            48  // Based on Timer CKCON settings

// There are SYSTEMCLOCK/TIMER_PRESCALER timer ticks per second, so
// SYSCLK/TIMER_PRESCALER/1000 timer ticks per millisecond.
#define TIMER_TICKS_PER_MS  SYSTEMCLOCK/TIMER_PRESCALER/1000

// Note: TIMER_TICKS_PER_MS should not exceed 255 (0xFF) for the 8-bit timer

#define AUX1     TIMER_TICKS_PER_MS
#define AUX2     -AUX1

#define TIMER0_RELOAD_HIGH       AUX2  // Reload value for Timer0 high byte

