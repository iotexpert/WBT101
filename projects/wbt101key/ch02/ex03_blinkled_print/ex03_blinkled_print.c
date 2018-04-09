/* Blink an LED with a frequency of 2 Hz */
/* Use WICED_BT_TRACE to print messages to the PUART */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
/* Timer will expire every 250ms so that the LED frequency will be 500ms = 2 Hz */
#define TIMEOUT_IN_MS       (250)

/*****************************    Variables   *****************************/
wiced_timer_t ms_timer;                 /* timer structure */

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void timer_cback( uint32_t arg );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    /* Send DEBUG messages over the PUART */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
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
            /* Start a timer to control blinking */
            if ( wiced_init_timer(&ms_timer, &timer_cback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER ) == WICED_SUCCESS )
			{
				wiced_start_timer( &ms_timer, TIMEOUT_IN_MS );
			}
            break;
        default:
            break;
    }
    return result;
}

/* The function invoked on timeout of the timer. */
void timer_cback( uint32_t arg )
{
	/* Read current set value for the LED pin and invert it */
	if(wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1) == GPIO_PIN_OUTPUT_HIGH)
	{
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW);
	    WICED_BT_TRACE("LED LOW\n\r");
	}
	else
	{
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH);
	    WICED_BT_TRACE("LED HIGH\n\r");
	}
}

