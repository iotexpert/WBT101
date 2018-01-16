/* Blink an LED with a frequency of 2 Hz */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
/* Timer will expire every 250ms so that the LED frequency will be 500ms = 2 Hz */
#define TIMEOUT_IN_MS       (250)
#define THREAD_PRIORITY     (5)
#define THREAD_STACK_SIZE   (10000)

/*****************************    Variables   *****************************/
wiced_thread_t* ledThreadHandle;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void ledThread( uint32_t arg );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL ); /* Register BT stack callback */
}


/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth  stack enabled */
        case BTM_ENABLED_EVT:
            /* Start a thread to control LED blinking */
            ledThreadHandle = wiced_rtos_create_thread();
            wiced_rtos_init_thread(ledThreadHandle, THREAD_PRIORITY, "ledThread", ledThread, THREAD_STACK_SIZE, NULL);
            break;
        default:
            break;
    }
    return result;
}


/* The thread function */
void ledThread( uint32_t arg )
{
	while(1)
	{
        /* Read current set value for the LED pin and invert it */
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));

        /* Delay 250ms */
        wiced_rtos_delay_milliseconds(TIMEOUT_IN_MS, ALLOW_THREAD_TO_SLEEP);
	}

}

