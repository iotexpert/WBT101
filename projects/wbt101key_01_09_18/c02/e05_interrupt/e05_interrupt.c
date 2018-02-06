/*Use a button interrupt to toggle LED whenever the button is pressed */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrrupt_handler(void *data, uint8_t port_pin);

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
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
            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrrupt_handler, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );
            break;
        default:
            break;
    }
    return result;
}


void gpio_interrrupt_handler(void *data, uint8_t port_pin)
{
   /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

	/* Toggle the LED state */
	wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));
}
