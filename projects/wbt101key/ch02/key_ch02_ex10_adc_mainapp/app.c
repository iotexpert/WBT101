#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_adc.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* Rating of thermistor balance resistor */
#define BALANCE_RESISTANCE	100000

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void app_task(uint32_t );


/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
	wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
	WICED_BT_TRACE("**** CYW20819 App Start **** \n\r");

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/**************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************    ***
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    switch( event )
    {
    case BTM_ENABLED_EVT:						// Bluetooth Controller and Host Stack Enabled

        if( WICED_BT_SUCCESS == p_event_data->enabled.status )
        {
			/* Turn on the ADC */
			wiced_hal_adc_init();
			wiced_hal_adc_set_input_range( ADC_RANGE_0_3P6V );
			
			/* The stack is safely up - create a thread to test out peripherals */
			wiced_thread_t* peripheral_test_thread = wiced_rtos_create_thread();

			wiced_rtos_init_thread(
					peripheral_test_thread,		// Thread handle
					7,                			// Priority (7 is low, 3 is high)
					"App Task",					// Name
					app_task,					// Function
					1024,						// Stack space for the app_task function to use
					NULL );						// Function argument (not used)
        }
        break;

    default:
        break;
    }

    return status;
}


/*******************************************************************************
* Function Name: void app_task(uint32_t)
********************************************************************************/
void app_task( uint32_t arg )
{
	uint32_t Vddio, Vmeas, Rtherm;

    while( 1 )
    {
    	Vddio = wiced_hal_adc_read_voltage( ADC_INPUT_VDDIO );
    	Vmeas = wiced_hal_adc_read_voltage( ADC_INPUT_P8 );

    	/*
    	 * Rtherm calculation for thermistor:
    	 *
    	 *			---		Vddio
    	 *			 \		^
    	 *			 /		|
    	 *	Rtherm	 \		|
    	 *			 /		|
    	 *			 \		|
    	 *			---		| Vmeas
    	 *			 /		| ^
    	 *			 \		| |
    	 *	BALANCE	 /		| |
     	 *		 	 \		| |
    	 *			 /		| |
    	 *			---		v v Ground
    	 */

    	Rtherm = ((Vddio - Vmeas) * BALANCE_RESISTANCE) / Vmeas;

		WICED_BT_TRACE( "Rtherm = %d Ohms\n\r", Rtherm );

        /* Send the thread to sleep for a specified number of milliseconds */
        wiced_rtos_delay_milliseconds( SLEEP_250MS, ALLOW_THREAD_TO_SLEEP );
    }
}

