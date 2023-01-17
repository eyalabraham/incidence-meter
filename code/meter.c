/*****************************************************************************
* meter.c
*
* AVR ATMega328P Incidence Meter
*
* Created: December 2022
*
*****************************************************************************/

#include    <stdint.h>
#include    <math.h>

#include    <avr/interrupt.h>
#include    <util/delay.h>

#include    "inc/util.h"
#include    "inc/sevseg.h"
#include    "inc/mpu6050.h"

/****************************************************************************
  Definitions
****************************************************************************/
#define     SEGMENT_MINUS       0b11111011
#define     SEGMENT_PLUS        0b11110011
#define     SEGMENT_DP          0b01111111
#define     SEGMENT_BLANK       0b11111111

#define     STATE_MEASURE       1
#define     STATE_CALIBRATE     2

/****************************************************************************
  Types
****************************************************************************/

/****************************************************************************
  Function prototypes
****************************************************************************/
void    light_test(void);
int     is_hold_zero_pressed(void);

/****************************************************************************
  Globals
****************************************************************************/
uint16_t    time_mark;
float       measured_angle;
float       display_angle;
float       calibration_offset;
float       hold_offset;
int         is_calibrated;
int         angle_positive;
int         angle_digit0;
int         angle_digit1;

int         mpu6050_ok = 0;
int         meter_state = STATE_MEASURE;
uint8_t     segment = SEGMENT_BLANK;

/* 7-segment LED segment bit assignments
*/
uint8_t seven_segment[10][2] = 
{
    //   '+1'          '8.'
    //      dcba    .gfedcba
    { 0b11111111, 0b11000000},  //  blank,  0
    { 0b11111100, 0b11111001},  //  1    ,  1
    { 0b11111111, 0b10100100},  //  blank,  2
    { 0b11111111, 0b10110000},  //  blank,  3
    { 0b11111111, 0b10011001},  //  blank,  4
    { 0b11111111, 0b10010010},  //  blank,  5
    { 0b11111111, 0b10000010},  //  blank,  6
    { 0b11111111, 0b11111000},  //  blank,  7
    { 0b11111111, 0b10000000},  //  blank,  8
    { 0b11111111, 0b10011000},  //  blank,  9
};

/* EEPROM calibration data structure
*/
struct calibration_t
{
    char    signature[3];   // Should be 'CAL' if calibration data is valid
    float   offset;         // Calibration offset
} calibration;

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    sev_seg_init();
    ioinit();
    mpu6050_ok = mpu6050_init();

    sei();

    light_test();

    /* Read calibration data from EEPROM
       and check if device is calibrated.
    */
    calibration_offset = 0.0;
    hold_offset = 0.0;
    is_calibrated = 0;

    retrieve_calibration((uint8_t*) &calibration, sizeof(calibration));
    if ( calibration.signature[0] == 'C' && 
         calibration.signature[1] == 'A' &&
         calibration.signature[2] == 'L'    )
    {
        calibration_offset = calibration.offset;
        is_calibrated = -1;
    }

    /* Main process loop
    */
    time_mark = get_global_time();
    while ( 1 )
    {
        if ( mpu6050_ok )
        /* process angle and display
        */
        {
            measured_angle = mpu6050_get_angle() - calibration_offset;

            display_angle  = measured_angle - hold_offset;

            /* Angle direction
            */
            if ( display_angle >= 0.0 )
            {
                angle_positive = -1;
            }
            else
            {
                angle_positive = 0;
                display_angle *= -1.0;
            }

            /* Angle auto-range
            */
            if ( display_angle < 1.0 )
            {
                angle_digit0 = (int)(display_angle * 10.0);
                angle_digit1 = 0;

                sevseg_set_digit(0, seven_segment[angle_digit0][1]);
                sevseg_set_digit(1, seven_segment[angle_digit1][1] & SEGMENT_DP);
                sevseg_set_digit(2, SEGMENT_BLANK & (angle_positive ? SEGMENT_PLUS : SEGMENT_MINUS));
            }
            else if ( display_angle < 10 )
            {
                angle_digit0 = (int)((display_angle - floor(display_angle)) * 10);
                angle_digit1 = (int)(display_angle);

                sevseg_set_digit(0, seven_segment[angle_digit0][1]);
                sevseg_set_digit(1, seven_segment[angle_digit1][1] & SEGMENT_DP);
                sevseg_set_digit(2, SEGMENT_BLANK & (angle_positive ? SEGMENT_PLUS : SEGMENT_MINUS));
            }
            else if ( display_angle < 90 )
            {
                angle_digit0 = (int)(display_angle);
                angle_digit1 = (int)(display_angle / 10.0);
                angle_digit0 -= angle_digit1 * 10;

                sevseg_set_digit(0, seven_segment[angle_digit0][1]);
                sevseg_set_digit(1, seven_segment[angle_digit1][1]);
                sevseg_set_digit(2, SEGMENT_BLANK & (angle_positive ? SEGMENT_PLUS : SEGMENT_MINUS));
            }
            else
            {
                /* Aanything >90 is not possible
                   and is considered an error.
                */
                sevseg_set_digit(0, SEGMENT_BLANK);
                sevseg_set_digit(1, SEGMENT_BLANK);
                sevseg_set_digit(2, SEGMENT_BLANK & (angle_positive ? SEGMENT_PLUS : SEGMENT_MINUS));
            }

            /* Meter calibration state.
            */
            if ( meter_state == STATE_CALIBRATE )
            {
                calibration_led_on();

                if ( is_hold_zero_pressed() )
                {
                    calibration_offset = mpu6050_get_angle();
                    hold_offset = 0.0;
                    calibration.signature[0] = 'C';
                    calibration.signature[1] = 'A';
                    calibration.signature[2] = 'L';
                    calibration.offset = calibration_offset;
                    save_calibration((uint8_t*) &calibration, sizeof(calibration));
                    is_calibrated = -1;
                    calibration_led_off();
                    meter_state = STATE_MEASURE;
                }
            }
            /* Normal measurement process and display.
               Need to check if zero/hold or calibrate
               buttons were pressed.
            */
            else if ( meter_state == STATE_MEASURE )
            {
                if ( is_hold_zero_pressed() )
                {
                    hold_offset += ((angle_positive ? +1.0 : -1.0) * display_angle);
                }
                else if ( is_calibrate() )
                {
                    meter_state = STATE_CALIBRATE;
                }

                /* Calibration indicator
                */
                if ( !is_calibrated && 
                    (uint16_t)(get_global_time() - time_mark) > RATE_4HZ )
                {
                    calibration_led_toggle();
                    time_mark = get_global_time();
                }
            }
            else
            {
                /* Undetermined state, do nothing */
            }
        }
        else
        /* Blink display with '99' if MPU-6050 is not functional
        */
        {
            if ( (uint16_t)(get_global_time() - time_mark) > RATE_4HZ )
            {
                if ( segment == SEGMENT_BLANK )
                    segment = seven_segment[9][1];
                else
                    segment = SEGMENT_BLANK;

                sevseg_set_digit(0, segment);
                sevseg_set_digit(1, segment);
                
                time_mark = get_global_time();
            }
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------------
* light_test()
*
*  Light LED 7-segment with test pattern
*
*  param:  none
*  return: none
*
*/
void light_test(void)
{
    sevseg_set_digit(0, 0b00000000);
    _delay_ms(1000);

    sevseg_set_digit(0, SEGMENT_BLANK);
    sevseg_set_digit(1, 0b00000000);
    _delay_ms(1000);

    sevseg_set_digit(1, SEGMENT_BLANK);
    sevseg_set_digit(2, 0b00000000);
    _delay_ms(1000);

    sevseg_set_digit(2, SEGMENT_BLANK);
}

/* ----------------------------------------------------------------------------
* is_hold_zero_pressed()
*
*  Inhibiting the zero/hold button is necessary in order
*  to prevent repeated action if the button is pressed
*  and not release.
*  Repeated button press detection will be enabled
*  one second after releasing the button.
*
*  param:  none
*  return: Zero/hold button state '-1'=pressed, '0'=not pressed
*
*/
int is_hold_zero_pressed(void)
{
    int             return_button_state = 0;

    static uint16_t button_time_mark;
    static int      button_pressed_state_inhibit = 0;

    /* Button was pressed after a released state:
       - return a press
    */
    if ( !button_pressed_state_inhibit && is_hold_zero() )
    {
        /* Latch the push button press state
        */
        button_pressed_state_inhibit = -1;
        return_button_state = -1;
    }
    /* Button is being held in a pressed position:
       - return a non-presed state
    */
    else if ( button_pressed_state_inhibit && is_hold_zero() )
    {
        return_button_state = 0;
    }
    /* Button was released:
       - return a non-pressed state and start a one-second timer.
       - after one second reset the inhibit state and accept new press states.
    */
    else if ( button_pressed_state_inhibit && !is_hold_zero() )
    {
        if ( button_time_mark )
        {
            if ( (uint16_t)(get_global_time() - button_time_mark) > ONE_SECOND )
            {
                button_pressed_state_inhibit = 0;
                button_time_mark = 0;
            }
        }
        else
        {
            button_time_mark = get_global_time();
        }

        return_button_state = 0;
    }

    return return_button_state;
}
