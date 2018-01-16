/* Whenever the kit button is pressed, read button data from shield and display to the UART */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
#define I2C_ADDRESS  (0x42)
/* I2C temperature register offset */
#define BUTTON_REG 	(0x06)

/*****************************    Variables   *****************************/
uint8_t i2cOffset = BUTTON_REG;

struct {
	uint8_t button_state;
	float temperature;
	float humidity;
	float light;
	float pot;
} __attribute__((packed)) i2cReadBuf;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrupt_callback(void *data, uint8_t port_pin);

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    /* Configure debug UART interface */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_bt_stack_init( bt_cback, NULL, NULL ); /* Register BT stack callback */
}


/* Callback function for BlueTooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:
            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrupt_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );
            /* Configure I2C block and pins*/
            wiced_hal_i2c_init();
            wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
            /* Write the offset for the temperature register */
            wiced_hal_i2c_write(&i2cOffset , sizeof(i2cOffset), I2C_ADDRESS);
            break;
        default:
            break;
    }
    return result;
}


void gpio_interrupt_callback(void *data, uint8_t port_pin)
{
	char buffer[100];

    /* Clear the gpio interrupt */
	wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

	/* Read I2C data */
	wiced_hal_i2c_read((char *) &i2cReadBuf , sizeof(i2cReadBuf), I2C_ADDRESS);

	/* Print data to UART */
	/* Since WICED_BT_TRACE doesn't support %f, we convert to scaled integers first */
	int32_t intTemp =  (int32_t) (10 * i2cReadBuf.temperature);
	int32_t intHum =   (int32_t) i2cReadBuf.humidity;
	int32_t intLight = (int32_t) i2cReadBuf.light;
    int32_t intPot =   (int32_t) (1000 * i2cReadBuf.pot);
    WICED_BT_TRACE("Temperature (0.1C): %d \tHumidity (%%): %d \tLight (lux): %d \tPOT (mV): %d\n\r", intTemp, intHum, intLight, intPot);
}
