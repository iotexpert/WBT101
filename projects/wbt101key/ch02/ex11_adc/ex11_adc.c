/* Measure Ambient Light Sensor voltage and send to UART every 500ms */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_hal_adc.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
#define TIMEOUT_IN_MS 	(250)
#define ADC_CHANNEL     (ADC_INPUT_P10)

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

	switch ( event )
	{
		/* BlueTooth  stack enabled */
		case BTM_ENABLED_EVT:
	       	/* Send DEBUG messages over the PUART */
			/* Note - for these pins to be connected to the serial device you must set the following switches: */
			/* SW5.1 OFF, SW5.2 ON, SW5.3 OFF, SW5.4 ON */
			wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

			/* Initialize the ADC */
			wiced_hal_adc_init( );

			/* Initialize and start the milliseconds timer */
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
    int16_t raw_val = 0;
    uint32_t voltage_val = 0;

    /* measure the voltage(raw count and voltage values) from the specified ADC channel */
    raw_val = wiced_hal_adc_read_raw_sample( ADC_CHANNEL );
    voltage_val = wiced_hal_adc_read_voltage( ADC_CHANNEL );

    /* Print results to the UART */
    WICED_BT_TRACE("Raw value (counts):\t %d\t\t", raw_val);
    WICED_BT_TRACE("Voltage (in mV):\t %d\r\n", voltage_val);
}
