/* Blink an LED with a frequency of 2 Hz */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"

extern wiced_platform_led_config_t platform_led[];
extern size_t led_count;

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);


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
            /* Initialize WICED functions */
            wiced_bt_app_init();



            //GJL 1 - if this section is uncommented then it works - i.e the LEDEs are both high
            /* Initialize LEDs and turn off by default */
//            uint32_t i;
//            for (i = 0; i < led_count; i++)
//            {
//                wiced_hal_gpio_configure_pin(*platform_led[i].gpio, platform_led[i].config, platform_led[i].default_state);
//            }



            //GJL 2 - if this is uncommented it does not work even if the for loop above is included
//            wiced_bt_app_init();


            break;
        default:
            break;
    }
    return result;
}
