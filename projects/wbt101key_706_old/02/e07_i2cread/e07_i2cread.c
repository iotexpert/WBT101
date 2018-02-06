/* Whenever the kit button is pressed, read button data from shield and display to the UART */

#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
/* The I2C address is 8 bit so need to left shift the 7-bit address by 1 */
#define I2C_ADDRESS  (0x42 <<1)
/* I2C temperature register offset */
#define BUTTON_REG 	(0x06)

/*****************************    Variables   *****************************/
/* HCI interface configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, 115200 },
    { 0, 0},
    NULL,
    NULL,
    NULL
};

uint8 i2cOffset = BUTTON_REG;

//GJL Temp - weather data is not used currently since float conversions are not supported
struct {
	uint8 button_state;
	float temperature;
	float humidity;
	float light;
	float pot;
} __attribute__((packed)) i2cReadBuf;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrrupt_handler(void *data, uint8_t port_pin);

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg ); /* Initialize HCI interface */
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
            /* Initialize WICED functions */
            wiced_bt_app_init();

            /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
            wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

            /* Configure the Button GPIO as an input with a resistive pull down and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, gpio_interrrupt_handler, NULL );
             wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_RISING_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE );
            /* Configure debug UART interface */
            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
        	WICED_BT_TRACE("\r"); // Go to start of line after trace init message

            /* Configure I2C block and pins*/
            wiced_hal_i2c_init(WICED_I2C_SDA_I2S_DOUT_PCM_OUT_SCL_I2S_DIN_PCM_IN);
            /* Write the offset for the temperature register */
            wiced_hal_i2c_write(&i2cOffset , 1, I2C_ADDRESS);

            break;
        default:
            break;
    }
    return result;
}


void gpio_interrrupt_handler(void *data, uint8_t port_pin)
{
	/* Clear the gpio interrupt */
	wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_BUTTON );

	/* Read I2C data */
	wiced_hal_i2c_read((char *) &i2cReadBuf , sizeof(i2cReadBuf), I2C_ADDRESS);

	//GJL TEMP this should be %f instead of %d
	/* Print data to UART */
    WICED_BT_TRACE("Button State: %02X\n\r", i2cReadBuf.button_state);

}
