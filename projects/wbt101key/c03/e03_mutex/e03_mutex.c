/* Blink an LED with a frequency of 2 Hz */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/

/* Comment out the following line to see what happens without the mutex */
#define USE_MUTEX

#define THREAD_PRIORITY     (5)
#define THREAD_STACK_SIZE   (10000)

/*****************************    Variables   *****************************/
#ifdef USE_MUTEX
wiced_mutex_t*  mutexHandle;
#endif
wiced_thread_t* led1ThreadHandle;
wiced_thread_t* led2ThreadHandle;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void led1Thread( uint32_t arg );
void led2Thread( uint32_t arg );

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
            /* Setup Mutex */
            #ifdef USE_MUTEX
            mutexHandle = wiced_rtos_create_mutex();
            wiced_rtos_init_mutex(mutexHandle);
            #endif
            /* Start threads to control LED blinking */
            led1ThreadHandle = wiced_rtos_create_thread();
            wiced_rtos_init_thread(led1ThreadHandle, THREAD_PRIORITY, "led1Thread", led1Thread, THREAD_STACK_SIZE, NULL);
            led2ThreadHandle = wiced_rtos_create_thread();
            wiced_rtos_init_thread(led2ThreadHandle, THREAD_PRIORITY, "led2Thread", led2Thread, THREAD_STACK_SIZE, NULL);
            break;
        default:
            break;
    }
    return result;
}


/* Thread function 1 */
void led1Thread( uint32_t arg )
{
	while(1)
	{
        #ifdef USE_MUTEX
        wiced_rtos_lock_mutex(mutexHandle);
        #endif
	    /* Loop while button is pressed */
	    while( 0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 ))
        {
            /* Read current set value for the LED pin and invert it */
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));
            /* Delay for LED blinking */
            wiced_rtos_delay_milliseconds(250, ALLOW_THREAD_TO_SLEEP);
        }
        #ifdef USE_MUTEX
        wiced_rtos_unlock_mutex(mutexHandle);
        #endif
	    wiced_rtos_delay_milliseconds(1, ALLOW_THREAD_TO_SLEEP);
	}
}


/* Thread function 2 */
void led2Thread( uint32_t arg )
{
    while(1)
    {
        #ifdef USE_MUTEX
        wiced_rtos_lock_mutex(mutexHandle);
        #endif
        /* Loop while button is pressed */
        while( 0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_2 ))
        {
            /* Read current set value for the LED pin and invert it */
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));
            /* Delay for LED blinking */
            wiced_rtos_delay_milliseconds(200, ALLOW_THREAD_TO_SLEEP);
        }
        #ifdef USE_MUTEX
        wiced_rtos_unlock_mutex(mutexHandle);
        #endif
        wiced_rtos_delay_milliseconds(1, ALLOW_THREAD_TO_SLEEP);
    }
}
