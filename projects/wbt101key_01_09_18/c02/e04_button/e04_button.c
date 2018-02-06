/* Monitor button every 100ms and set the LED based on the button's state */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"

/*****************************    Constants   *****************************/
/* Timer will expire every 100ms so that we can examine the button state */
#define TIMEOUT_IN_MS (100)

/*****************************    Variables   *****************************/
wiced_timer_t ms_timer;               	/* timer structure */

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void timer_cback( uint32_t arg );

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
            /* Start a timer to monitor the button */
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
    uint32_t bval;
    bval = wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 );
    /* Read state of button and set LED based on the button state */
	if(0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 )) // Button is active low
	{
		/* Drive LED HIGH */
		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH);
	}
	else
	{
		/* Drive LED LOW */
		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW);
	}
}

