/* Blink an LED with a frequency of 2 Hz */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
/* Timer will expire every 250ms so that the LED frequency will be 500ms = 2 Hz */
#define TIMEOUT_IN_MS       (250)

/*****************************    Variables   *****************************/
wiced_rtos_timer_t* ledTimerHandle;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void ledTimer( int32_t arg );

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
            ledTimerHandle = wiced_rtos_create_timer();
            wiced_rtos_init_timer(ledTimerHandle, TIMEOUT_IN_MS, ledTimer, NULL);
            wiced_rtos_start_timer(ledTimerHandle);
            break;
        default:
            break;
    }
    return result;
}


/* The thread function */
void ledTimer( int32_t arg )
{
    /* Read current set value for the LED pin and invert it */
    wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));
}

