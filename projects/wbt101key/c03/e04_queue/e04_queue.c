/* Toggle the state of an LED when the button is pressed using a semaphore  */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
#define THREAD_PRIORITY     (5)
#define THREAD_STACK_SIZE   (10000)
#define MESSAGE_SIZE        (4)
#define QUEUE_LENGTH        (10)

/*****************************    Variables   *****************************/
wiced_queue_t* queueHandle;
wiced_thread_t* ledThreadHandle;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrupt_callback(void *data, uint8_t port_pin);
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
            /* Setup the semaphore */
            queueHandle = wiced_rtos_create_queue();
            wiced_rtos_init_queue(queueHandle, "queue", MESSAGE_SIZE, QUEUE_LENGTH);

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrupt_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );

            /* Start a thread to control LED blinking */
            ledThreadHandle = wiced_rtos_create_thread();
            wiced_rtos_init_thread(ledThreadHandle, THREAD_PRIORITY, "ledThread", ledThread, THREAD_STACK_SIZE, NULL);
            break;
        default:
            break;
    }
    return result;
}


void gpio_interrupt_callback(void *data, uint8_t port_pin)
{
   /*  Number of times to blink the LED */
   static uint32_t blink = 0;

   /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

   /* Increment value and push  to the queue */
   blink++;
   wiced_rtos_push_to_queue( queueHandle,  &blink, WICED_NO_WAIT );
}


/* The thread function */
void ledThread( uint32_t arg )
{
	uint32_t loop;
	uint32_t blinkValue;

    while(1)
	{
        /* Wait for button press semaphore */
	    wiced_rtos_pop_from_queue( queueHandle, &blinkValue, WICED_WAIT_FOREVER );

	    /* Blink the number of times indicated by the queue */
	    for( loop = 0; loop < blinkValue; loop++)
	    {
	        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
	        wiced_rtos_delay_milliseconds( 250, ALLOW_THREAD_TO_SLEEP );
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
            wiced_rtos_delay_milliseconds( 250, ALLOW_THREAD_TO_SLEEP );
	    }
	    /* Wait 1 second between button presses */
        wiced_rtos_delay_milliseconds( 1000, ALLOW_THREAD_TO_SLEEP );
	}

}

