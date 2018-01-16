/*Use a button interrupt to send data on the PUART */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_puart.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void rx_interrupt_callback(void* unused);

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
            /* Setup UART */
            wiced_hal_puart_init( );
            wiced_hal_puart_flow_off( );
            wiced_hal_puart_set_baudrate( 115200 );
            /* Register the interrupt for Rx events */
            wiced_hal_puart_register_interrupt(rx_interrupt_callback);
            /* Set watermark level to 1 to receive interrupt up on receiving each byte */
            wiced_hal_puart_set_watermark_level(1);
            /* Enable Rx traffic */
            wiced_hal_puart_enable_rx();
            break;
        default:
            break;
    }
    return result;
}


void rx_interrupt_callback(void* unused)
{
    uint8_t  readbyte;

    /* Read one byte from the buffer and reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt( );

    /* Turn LED ON/OFF bases on character received */
    if( readbyte == '1' )
    {
        /* Drive LED HIGH */
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH);
    }
    if( readbyte == '0' )
    {
        /* Drive LED LOW */
         wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW);
    }
}
