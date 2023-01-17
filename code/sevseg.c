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

#include    <avr/io.h>

#include    "inc/sevseg.h"

/****************************************************************************
  Types and definitions
****************************************************************************/
#define     DIGIT_SELECT_MASK   ((1 << DIGIT_COUNT)-1)

/****************************************************************************
  Local function prototypes
****************************************************************************/

/****************************************************************************
  Module globals
****************************************************************************/
static uint8_t  digit_bitmaps[DIGIT_COUNT];
static uint8_t  digit_index;
static uint8_t  shift;

/****************************************************************************
  Module functions
****************************************************************************/

/* ----------------------------------------------------------------------------
 * sev_seg_init()
 *
 *  Intialize the seven segment multiplexer.
 *
 *  param:  none
 *  return: Digit count.
 *
 */
int sev_seg_init(void)
{
    int     i;

    digit_index = 1;
    shift = 0;
    for ( i = 0; i < DIGIT_COUNT; i++ )
    {
        digit_bitmaps[i] = 0xff;
    }

    return DIGIT_COUNT;
}

/* ----------------------------------------------------------------------------
 * sevseg_multiplexer_update()
 *
 *  Update the next digit in the series of multiplexed digits.
 *  Call periodically from a loop or interrupt handler.
 *  Assumes IO ports are configures as:
 *    PORT B b.0, b.1, b.2 ... are digit selectors.
 *    PORT D b.0 to b.8 are segment bits 'a' to 'g' and decimal point.
 *
 *  param:  none
 *  return: none
 *
 */
void sevseg_multiplexer_update(void)
{
    /* Select digit and activate segments
    */
    PORTB &= ~DIGIT_SELECT_MASK;
    PORTD = digit_bitmaps[shift];
    PORTB |= digit_index;

    /* Shift multiplexer
    */
    digit_index <<= 1;
    shift++;
    if ( shift >= DIGIT_COUNT )
    {
        digit_index = 1;
        shift = 0;
    }

}

/* ----------------------------------------------------------------------------
 * sevseg_set_digit()
 *
 *  Update a digit's segment bitmap.
 *  The segment bitmap is defined as bits 0 through 6 for segments 'a' to 'g'
 *  and bit 7 for the decimal point.
 *
 *  param:  Digit index to update and the segment's bitmap.
 *  return: -1 bad digit index, 0 for ok
 *
 */
int sevseg_set_digit(int digit, uint8_t segments)
{
    if ( digit >= DIGIT_COUNT || digit < 0 )
        return -1;

    digit_bitmaps[digit] = segments;

    return 0;
}
