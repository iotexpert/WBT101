/* Change a PWM duty cycle to change brightness of an LED */

#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_hal_aclk.h"
#include "wiced_hal_pwm.h"

/*****************************    Constants   *****************************/
/* Timer will expire every 250ms so that the LED frequency will be 500ms = 2 Hz */
/* Brightness will change every 100ms */
#define TIMEOUT_IN_MS (10)
/* Frequency is in Hz so this gives us 100 kHz */
#define CLK_FREQ                (100000)
/* The LED is on GPIO WICED_P26 which is connected to PWM0  */
#define PWM_CHANNEL             (PWM0)
/* The PWM starts at the init value and counts up to 0x3FF. It switches state at the toggle value */
/* These values will start us at a period of 100 (i.e. 100 KHz/100 = 100Hz) and a 50% duty cycle */
#define PWM_MAX					(0x3FF)
#define PWM_INIT				(PWM_MAX - 99)
#define PWM_TOGGLE				(PWM_MAX - 50)

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

wiced_timer_t ms_timer;        /* timer structure */
uint16 pwmInit   = PWM_INIT;
uint16 pwmToggle = PWM_TOGGLE;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void timer_cback( uint32_t arg );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg ); /* Initialize HCI interface */
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

            /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
            wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

            wiced_hal_aclk_enable(CLK_FREQ, ACLK1, ACLK_FREQ_24_MHZ);
            wiced_hal_pwm_start(PWM_CHANNEL, PMU_CLK, pwmToggle, pwmInit, 0);

            /* Start a timer to change the PWMs duty cycle */
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
	pwmToggle++; /* Increase duty cycle by 1% (1 count out of 100) */
	if(pwmToggle > PWM_MAX) /* Reset to 0% duty cycle once we reach 100% */
	{
		pwmToggle = PWM_INIT;
	}
	pwm_transitionToSubstituteValues(PWM_CHANNEL, pwmToggle, pwmInit);
}

