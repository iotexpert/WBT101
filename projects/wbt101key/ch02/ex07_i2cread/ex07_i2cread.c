/* Whenever BUTTON_1 is pressed, read data from the shield and print to PUART */

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
#define BUTTON_VALUE_REG            (0x06)
#define TEMPERATURE_VALUE_REG       (0x07)


/*****************************     Macros     *****************************/

/* Convert float into integer-dot-integer values */
#define FABS(f)                     ((f<0.0)?-f:f)
#define INTEGER(f)                  ((int)f)
#define FRACTION(f)                 ((int)((FABS(f)-INTEGER(FABS(f)))*10))


/*****************************    Variables   *****************************/


/*****************************    Function Prototypes   *******************/

wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void led_control( uint32_t arg );
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

    char i2cWriteBuf[1];                                            // 1-byte write buffer

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:

            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            WICED_BT_TRACE( "*** ex07_i2cread ***\n\r" );

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );

            /* Configure I2C block */
            wiced_hal_i2c_init();
            wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );

            /* Write the offset TEMPERATURE value register to be controlled over I2C (just one byte) */
            i2cWriteBuf[0] = TEMPERATURE_VALUE_REG;
            wiced_hal_i2c_write( i2cWriteBuf , 1, I2C_ADDRESS );

            break;

        default:
            break;
    }
    return result;
}


/* Interrupt callback function for BUTTON_1 */
void button_callback(void *data, uint8_t port_pin)
{
    struct
    {
        float temperature;
        float humidity;
        float light;
        float pot;
    } __attribute__((packed)) i2cReadBuf;   // I2C read data

    /* Read the weather data */
    wiced_hal_i2c_read( (char *) &i2cReadBuf , sizeof( i2cReadBuf ), I2C_ADDRESS );

    /* Print data to UART */
    /* Since WICED_BT_TRACE doesn't support %f, we convert to scaled integers first */
    WICED_BT_TRACE( "Temperature: %3d.%1d C  ",  INTEGER(i2cReadBuf.temperature), FRACTION(i2cReadBuf.temperature) );
    WICED_BT_TRACE( "Humidity: %3d.%1d %%  ",    INTEGER(i2cReadBuf.humidity),    FRACTION(i2cReadBuf.humidity) );
    WICED_BT_TRACE( "Light: %5d.%1d lux  ",      INTEGER(i2cReadBuf.light),       FRACTION(i2cReadBuf.light) );
    WICED_BT_TRACE( "Voltage: %d.%1d V\n\r",     INTEGER(i2cReadBuf.pot),         FRACTION(i2cReadBuf.pot) );
}
