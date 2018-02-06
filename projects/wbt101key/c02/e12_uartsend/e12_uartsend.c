/*Use a button interrupt to send data on the PUART */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_puart.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrrupt_callback(void *data, uint8_t port_pin);

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
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrrupt_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );

            /* Setup UART and enable Tx */
            wiced_hal_puart_init( );
            wiced_hal_puart_flow_off( );
            wiced_hal_puart_set_baudrate( 115200 );
            wiced_hal_puart_enable_tx( );
            wiced_hal_puart_print("Press button 1 to increment the value\n\r");
            wiced_hal_puart_print("Value = 0\r");
            break;
        default:
            break;
    }
    return result;
}


void gpio_interrrupt_callback(void *data, uint8_t port_pin)
{
   static uint8_t value = 0;
   char printString[10] = {0};

    /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

   /* Increment the value to print */
   value++;
   if(value > 9)
   {
       value = 0;
   }

   /* Print value to the screen */
   wiced_hal_puart_print("Value = ");
   /* Add '0' to the value to get the ASCII equivalent of the number */
   wiced_hal_puart_write(value+'0');
   wiced_hal_puart_print("\r");
}
