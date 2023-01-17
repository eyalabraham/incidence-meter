/*****************************************************************************
* sevseg.h
*
* Driver hearder for Seven segment display.
*
* This is driver does not rely on interrupts. Multiplexing can be used
* with a periodic call in a simple loop or from an interrupr routine
*
* Created: December 2022
*
*****************************************************************************/

#ifndef __SEVSEG_H__
#define __SEVSEG_H__

#include    <stdint.h>

/****************************************************************************
  Definitions
****************************************************************************/
#define     DIGIT_COUNT     3   // Maximum 8 seven segment digits

/****************************************************************************
  Function prototypes
****************************************************************************/
int     sev_seg_init(void);
void    sevseg_multiplexer_update(void);
int     sevseg_set_digit(int digit, uint8_t segments);

#endif /* __SEVSEG_H__ */