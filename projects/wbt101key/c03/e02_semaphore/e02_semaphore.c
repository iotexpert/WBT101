/* Toggle the state of an LED when the button is pressed using a semaphore  */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
#define THREAD_PRIORITY     (5)
#define THREAD_STACK_SIZE   (10000)

/*****************************    Variables   *****************************/
wiced_semaphore_t* semaphoreHandle;
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
            semaphoreHandle = wiced_rtos_create_semaphore();
            wiced_rtos_init_semaphore(semaphoreHandle);

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
   /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

   /* Set the semaphore */
   wiced_rtos_set_semaphore(semaphoreHandle);

}


/* The thread function */
void ledThread( uint32_t arg )
{
	while(1)
	{
        /* Wait for button press semaphore */
	    wiced_rtos_get_semaphore(semaphoreHandle, WICED_WAIT_FOREVER);

	    /* Read current set value for the LED pin and invert it */
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, !wiced_hal_gpio_get_pin_output(WICED_GPIO_PIN_LED_1));
	}

}

