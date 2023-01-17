/*****************************************************************************
* util.h
*
* Utility functions
*
* Created: December 2022
*
*****************************************************************************/

#ifndef __UTIL_H__
#define __UTIL_H__

#include    <stdint.h>

/****************************************************************************
  Definitions
****************************************************************************/
/* Timer0 time base
 */
#if (F_CPU == 8000000UL)
#define     RATE_1HZ        150             // Equivalent timer ticks
#define     RATE_2HZ        75
#define     RATE_4HZ        38
#define     ONE_SECOND      RATE_1HZ
#else
#warning "Device clock must be 8MHz"
#endif

#define     CALIBRATE_LED   0b00001000      // '1'=LED off, '0'=LED on
#define     SW_CALIBRATE    0b00100000      // Toggle calibration mode
#define     SW_HOLD_ZERO    0b00010000      // Hold zero reference measurement

/****************************************************************************
  Function prototypes
****************************************************************************/
void     reset(void) __attribute__((naked)) __attribute__((section(".init3")));
void     ioinit(void);
uint16_t get_global_time(void);
void     calibration_led_on(void);
void     calibration_led_off(void);
void     calibration_led_toggle(void);
int      is_calibrate(void);
int      is_hold_zero(void);
void     retrieve_calibration(uint8_t *data_bytes, int byte_count);
void     save_calibration(uint8_t *data_bytes, int byte_count);

#endif /* __UTIL_H__ */
