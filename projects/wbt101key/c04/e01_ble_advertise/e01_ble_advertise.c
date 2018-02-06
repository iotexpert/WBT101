/* Setup the BLE to advertise. Include variable data in the advertisement
 * packet that is updated each time the user button is pressed. */

#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"

/*****************************    Constants   *****************************/
extern const wiced_bt_cfg_settings_t bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t bt_cfg_buf_pools[];

/*****************************    Variables   *****************************/
/* HCI interface configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, 115200 },
    { 0, 0},
    NULL,
    NULL,
    NULL
};

/* Manufacturer data - we will increment this with each button press */
uint8_t manuf_data = 0x01;

/*****************************    Function Prototypes   *******************/
static wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void application_init( void );
static void bt_set_advertisement_data( void );
static void gpio_interrrupt_handler(void *data, uint8_t port_pin);

/*****************************    Functions   *****************************/
/**************************************************************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg ); /* Initialize HCI interface */
    wiced_bt_stack_init( bt_cback, &bt_cfg_settings, bt_cfg_buf_pools );
}

/**************************************************************************/
/* This function is executed in the BTM_ENABLED_EVT management callback. */
void application_init( void )
{
    wiced_result_t result;

    /* Initialize WICED functions */
    wiced_bt_app_init();

    /* Send DEBUG messages over the PUART */
    /* Note - for these pins to be connected to the serial device you must set the following switches: */
    /* SW5.1 OFF, SW5.2 ON, SW5.3 OFF, SW5.4 ON */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
    WICED_BT_TRACE("\r"); // Go to start of line after trace init message

    /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
    wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    /* Configure the Button GPIO as an input with a resistive pull down and interrupt on rising edge */
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, gpio_interrrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_RISING_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE );

    /* Set up BLE advertising parameters */
    bt_set_advertisement_data();
    /* Start non-connectable high duty cycle advertising */
    /* Note: 2nd and third parameters are not relevant for non-connectable advertising */
    result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, NULL, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n\r", result );
}

/**************************************************************************/
/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            application_init();
            break;
        default:
            break;
    }
    return result;
}

void bt_set_advertisement_data( void )
{
	/* TX power for advertisement packets , 10dBm */
    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_TX_POWER;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = ( uint8_t* ) &bt_cfg_settings.max_pwr_db_val;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (char *)bt_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )bt_cfg_settings.device_name ;
    num_elem++;

    adv_elem[num_elem].advert_type  =  BTM_BLE_ADVERT_TYPE_MANUFACTURER;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &manuf_data;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}


/**************************************************************************/
/* Interrupt handler for button press. This incrementes the manuf_data advertising parameter. */
void gpio_interrrupt_handler(void *data, uint8_t port_pin)
{
	wiced_result_t result;

	/* Clear the GPIO interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_BUTTON );

   /* Update advertising parameters */
   manuf_data++;
   bt_set_advertisement_data();

   /* Re-start advertising */
   result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, NULL, NULL );
   WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n\r", result );
}
