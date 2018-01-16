/* Whenever the button is pressed, write data to cycle through the
 * CapSense LEDs on the shield board */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_i2c.h"

/*****************************    Constants   *****************************/
#define I2C_ADDRESS  (0x42)
/* I2C register locations */
#define LED_CONTROL_REG 	(0x05)
#define LED_VALUE_REG     	(0x04)

/*****************************    Variables   *****************************/
uint8_t I2Cdata[2];

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void gpio_interrupt_callback(void *data, uint8_t port_pin);

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
             wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrupt_callback, NULL );
             wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );
            /* Configure I2C block */
             wiced_hal_i2c_init();
             wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
            /* Write the offset and the bit for the shield LEDs to be controlled over I2C */
            I2Cdata[0] = LED_CONTROL_REG;
            I2Cdata[1] = 0x01;
            wiced_hal_i2c_write(I2Cdata , sizeof(I2Cdata), I2C_ADDRESS);
            /* Set the offset for the LED value register to be used in all future transactions */
            I2Cdata[0] = LED_VALUE_REG;
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

   /* Update I2C register */
   wiced_hal_i2c_write(I2Cdata , sizeof(I2Cdata), I2C_ADDRESS);
   I2Cdata[1] <<= 1; /* Shift active LED bit left one */
   if(I2Cdata[1] > 0x08)
   {
	   I2Cdata[1] = 0x01;
   }
}
