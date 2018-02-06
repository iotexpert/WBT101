/* Whenever the button is pressed, write data to toggle LEDs on the shield board */

#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_hal_i2c.h"

/*****************************    Constants   *****************************/
#define I2C_ADDRESS  (0x42 <<1) /* The I2C address is 8 bit so need to left shift the 7-bit address by 1 */
/* I2C register locations */
#define LED_CONTROL_REG 	(0x05)
#define LED_VALUE_REG     	(0x04)

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

uint8 I2Cdata[2];

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
        /* BlueTooth  stack enabled */
        case BTM_ENABLED_EVT:
            /* Initialize WICED functions */
            wiced_bt_app_init();

            /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
            wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

            /* Configure the Button GPIO as an input with a resistive pull down and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, gpio_interrrupt_handler, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_RISING_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE );
            /* Configure I2C block and pins*/
            wiced_hal_i2c_init(WICED_I2C_SDA_I2S_DOUT_PCM_OUT_SCL_I2S_DIN_PCM_IN);
            /* Write the offset and the bit for the shield LEDs to be controlled over I2C */
            I2Cdata[0] = LED_CONTROL_REG;
            I2Cdata[1] = 0x01;
            wiced_hal_i2c_write(I2Cdata , 2, I2C_ADDRESS);
            /* Set the offset for the LED value register to be used in all future transactions */
            I2Cdata[0] = LED_VALUE_REG;
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

   /* Update I2C register */
   wiced_hal_i2c_write(I2Cdata , 2, I2C_ADDRESS);
   I2Cdata[1] <<= 1; /* Shift active LED bit left one */
   if(I2Cdata[1] > 0x08)
   {
	   I2Cdata[1] = 0x01;
   }
}
