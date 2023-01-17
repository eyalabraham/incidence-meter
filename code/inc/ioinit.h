/*****************************************************************************
* ioinit.h
*
* IO initialization definitions
*
* Created: December 2022
*
*****************************************************************************/

#ifndef __IOINIT_H__
#define __IOINIT_H__

/*
 *
 * Port B bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' Right digit   [8.]
 * |  |  |  |  |  |  +------ 'o' Middile digit [8.]
 * |  |  |  |  |  +--------- 'o' Left digit    [+1]
 * |  |  |  |  +------------ 'o' Calibration indicator LED
 * |  |  |  +--------------- 'i' Hold/zero measurement push-button
 * |  |  +------------------ 'i' Calibrate push-button
 * |  +--------------------- 'i'
 * +------------------------ 'i'
 *
 * Port C bit assignment
 *
 *    b6 b5 b4 b3 b2 b1 b0
 *    |  |  |  |  |  |  |
 *    |  |  |  |  |  |  +--- 'i'
 *    |  |  |  |  |  +------ 'i'
 *    |  |  |  |  +--------- 'i'
 *    |  |  |  +------------ 'i'
 *    |  |  +--------------- 'i' SDA
 *    |  +------------------ 'i' SCL
 *    +--------------------- 'i' Reset input
 *
 * Port D bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' Segment 'a'
 * |  |  |  |  |  |  +------ 'o' Segment 'b'
 * |  |  |  |  |  +--------- 'o' Segment 'c'
 * |  |  |  |  +------------ 'o' Segment 'd'
 * |  |  |  +--------------- 'o' Segment 'e'
 * |  |  +------------------ 'o' Segment 'f'
 * |  +--------------------- 'o' Segment 'g'
 * +------------------------ 'o' Segment 'dp'
 *
 */
 
/****************************************************************************
  Definitions
****************************************************************************/
/* IO port B initialization
 */
#define     PB_DDR_INIT     0b00001111  // Port data direction
#define     PB_PUP_INIT     0b00110000  // Port input pin pull-up
#define     PB_INIT         0b00001000  // Port initial values

/* IO port C initialization
 */
#define     PC_DDR_INIT     0b00000000  // Port data direction
#define     PC_PUP_INIT     0b00000000  // Port input pin pull-up
#define     PC_INIT         0b00000000  // Port initial values

/* IO port D initialization
 */
#define     PD_DDR_INIT     0b11111111  // Port data direction
#define     PD_PUP_INIT     0b00000000  // Port input pin pull-up
#define     PD_INIT         0b11111111  // Port initial values

/* Timer0 initialization
 * With 8MHz internal clock: 150Hz / 6.7 [mSec]
 */
#define     TCCR0A_INIT     0b00000010  // Normal mode, CTC to OCRA
#define     TCCR0B_INIT     0b00000101  // Mode-0 timer, Clk/1024
#define     OCR0A_INIT      52          // OCR0A value for ~150Hz interrupt
#define     TIMSK_INIT      0b00000010  // Enable Timer0 OCRA match interrupt

#endif /* __IOINIT_H__ */
