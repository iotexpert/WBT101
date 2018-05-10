/* Whenever BUTTON_1 is pressed, write data to cycle through the
 * CapSense LEDs on the shield board */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"


/*****************************    Constants   *****************************/

/* Address of the I2C slave on the shield */
#define I2C_ADDRESS                 (0x42)

/* I2C register locations inside the PSoC */
#define LED_VALUE_REG               (0x04)
#define LED_CONTROL_REG             (0x05)


/*****************************    Variables   *****************************/

uint8_t i2cWriteBuf[2];                 // I2C write data


/*****************************    Function Prototypes   *******************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void button_callback(void *data, uint8_t port_pin );


/*****************************    Functions   *****************************/

/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL );             // Register BT stack callback
}


/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:

            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            WICED_BT_TRACE( "*** ex06_i2cwrite ***\n\r" );

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );

            /* Configure I2C block */
            wiced_hal_i2c_init();
            wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );

            /* Write the offset and the bit for the shield LEDs to be controlled over I2C */
            i2cWriteBuf[0] = LED_CONTROL_REG;
            i2cWriteBuf[1] = 0x01;
            wiced_hal_i2c_write( i2cWriteBuf , sizeof( i2cWriteBuf ), I2C_ADDRESS );

            /* Set the offset for the LED value register and select the first LED */
            i2cWriteBuf[0] = LED_VALUE_REG;
            i2cWriteBuf[1] = 1;

            break;

        default:
            break;
    }
    return result;
}


/* Interrupt callback function for BUTTON_1 */
void button_callback(void *data, uint8_t port_pin)
{
    /* Update I2C register */
    wiced_hal_i2c_write( i2cWriteBuf , sizeof( i2cWriteBuf ), I2C_ADDRESS );

    WICED_BT_TRACE( "Next!\n\r" );

    i2cWriteBuf[1] <<= 1;               // Shift active LED bit left one
    if( i2cWriteBuf[1] > 0x08 )
    {
        i2cWriteBuf[1] = 0x01;          // Reset to the first bit
    }
}
