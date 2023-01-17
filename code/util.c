/*****************************************************************************
* util.c
*
* Utility functions
*
* Created: December 2022
*
*****************************************************************************/

#include    <stdint.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <util/delay.h>

#include    "inc/ioinit.h"
#include    "inc/sevseg.h"
#include    "inc/i2c_drv.h"
#include    "inc/util.h"

/****************************************************************************
  Types and definitions
****************************************************************************/
#define     SW_DEBOUNCE     10              // mSec debounce time

/****************************************************************************
  Local function prototypes
****************************************************************************/

/****************************************************************************
  Module globals
****************************************************************************/
static volatile uint16_t   global_counter = 0;     // Global time base

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0;  // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  Initialize IO interfaces.
 *
 */
void ioinit(void)
{
    /* Reconfigure system clock scaler
     */
    CLKPR = 0x80;   // Enable scaler        (sec 8.12.2)
    CLKPR = 0x00;   // Change clock scaler

    /* General IO pins
     */
    DDRB  = PB_DDR_INIT;
    PORTB = PB_INIT | PB_PUP_INIT;

    DDRC  = PC_DDR_INIT;
    PORTC = PC_INIT | PC_PUP_INIT;

    DDRD  = PD_DDR_INIT;
    PORTD = PD_INIT | PD_PUP_INIT;

    /* Timer0
     */
    TCNT0 = 0;
    TCCR0A = TCCR0A_INIT;
    TCCR0B = TCCR0B_INIT;
    OCR0A  = OCR0A_INIT;
    TIMSK0 = TIMSK_INIT;

    /* TWI used by MPU-6050 and initialized in mpu6050.c
    */
}

/* ----------------------------------------------------------------------------
 * get_global_time()
 *
 *  Return the value of the global timer tick counter
 *
 *  param:  none
 *  return: counter value
 *
 */
uint16_t get_global_time(void)
{
    return global_counter;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger approximately every 6.7mSec @ 8MHz clock.
 * The ISR increments a global 16-bit time variable that will overflow (cycle back through 0)
 * approximately every 7 minutes.
 *
 */
ISR(TIMER0_COMPA_vect)
{
    global_counter++;
    sevseg_multiplexer_update();
}

/* ----------------------------------------------------------------------------
 * calibration_led_on()
 *
 *  Turn on LED
 *
 *  param:  none
 *  return: none
 *
 */
void calibration_led_on(void)
{
    PORTB &= ~CALIBRATE_LED;
}

/* ----------------------------------------------------------------------------
 * calibration_led_off()
 *
 *  Turn LED off
 *
 *  param:  none
 *  return: none
 *
 */
void calibration_led_off(void)
{
    PORTB |= CALIBRATE_LED;
}

/* ----------------------------------------------------------------------------
 * calibration_led_toggle()
 *
 *  Toggle LED state on->off or off->on
 *
 *  param:  none
 *  return: none
 *
 */
void calibration_led_toggle(void)
{
    PORTB ^= CALIBRATE_LED;
}

/* ----------------------------------------------------------------------------
 * is_calibrate()
 *
 *  Check calibration push-button state.
 *  With switch debounce.
 *
 *  param:  none
 *  return: '0'=not pressed, '-1'=pressed
 *
 */
int is_calibrate(void)
{
    int     state1, state2;

    state1 = ((PINB & SW_CALIBRATE) ? 0 : -1);
    _delay_ms(SW_DEBOUNCE);
    state2 = ((PINB & SW_CALIBRATE) ? 0 : -1);

    if ( state1 == state2 )
        return state2;
    else
        return 0;
}

/* ----------------------------------------------------------------------------
 * is_hold_zero()
 *
 *  Check hold zero push-button state
 *  With switch debounce.
 *
 *  param:  none
 *  return: '0'=not pressed, '-1'=pressed
 *
 */
int is_hold_zero(void)
{
    int     state1, state2;

    state1 = ((PINB & SW_HOLD_ZERO) ? 0 : -1);
    _delay_ms(SW_DEBOUNCE);
    state2 = ((PINB & SW_HOLD_ZERO) ? 0 : -1);

    if ( state1 == state2 )
        return state2;
    else
        return 0;
}

/* ----------------------------------------------------------------------------
 * retrieve_calibration()
 *
 *  Retrieve calibration data from EEPROM
 *  Only address 0 through 255.
 *
 */
void retrieve_calibration(uint8_t *data_bytes, int byte_count)
{
    uint8_t     eeprom_address;

    if ( byte_count > 255 ) 
        return;

    EEARH = 0;
    EEARL = 0;

    /* Make sure no writing is in progress
     * should not happen!
     */
    while ( bit_is_set(EECR, EEPE) ) {};

    /* Read calibration data
     */
    for ( eeprom_address = 0; eeprom_address < byte_count; eeprom_address++ )
    {
        EEARL = eeprom_address;
        EECR |= 0b00000001; // Set EERE to read EEPROM
        data_bytes[eeprom_address] = EEDR;
    }
}

/* ----------------------------------------------------------------------------
 * save_calibration()
 *
 *  Store calibration data to EEPROM.
 *  Only address 0 through 255.
 *
 *  [data sheet] Using Atomic Byte Programming is the simplest mode. When writing a byte to the EEPROM,
 *  the user must write the address into register EEAR and data into register EEDR. If the EEPMn bits
 *  are zero, writing EEPE (within four cycles after EEMPE is written) will trigger the erase/write
 *  operation. Both the erase and write cycle are done in one operation. The EEPE bit remains set
 *  until the erase and write operations are completed. While the device is busy with programming,
 *  it is not possible to do any other EEPROM operations.
 *
 */
void save_calibration(uint8_t *data_bytes, int byte_count)
{
    uint8_t     eeprom_address;

    if ( byte_count > 255 ) 
        return;

    EEARH = 0;
    EEARL = 0;

    /* Write calibration data
     */
    for ( eeprom_address = 0; eeprom_address < byte_count; eeprom_address++ )
    {
        EEARL = eeprom_address;
        EEDR  = data_bytes[eeprom_address];
        EECR &= 0b11001111; // Atomic write
        EECR |= 0b00000100; // Set EEMPE
        EECR |= 0b00000010; // Set EEPE to start programming
        while ( bit_is_set(EECR, EEPE) ) {};
    }
}
