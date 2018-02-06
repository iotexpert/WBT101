/* Change a PWM duty cycle to change brightness of an LED */

#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_timer.h"
#include "wiced_hal_pwm.h"

/*****************************    Constants   *****************************/
/* Brightness will change every 10ms */
#define TIMEOUT_IN_MS (10)

#define PWM_CHANNEL             (PWM0)
/* The PWM starts at the init value and counts up to 0xFFFF. Then it wraps back around to the init value
 * The output of the PWM starts low and switches high at the toggle value */
/* These values will cause the PWM to count from (0xFFFF - 99) to 0xFFFF (i.e. period = 100)
 * with a duty cycle of 0xFFFF - 50 (i.e. a 50% duty cycle). */
#define PWM_MAX					(0xFFFF)
#define PWM_INIT				(PWM_MAX - 99)
#define PWM_TOGGLE				(PWM_MAX - 50)

/*****************************    Variables   *****************************/
wiced_timer_t ms_timer;        /* timer structure */
uint16_t pwmInit   = PWM_INIT;
uint16_t pwmToggle = PWM_TOGGLE;

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

    switch( event )
    {
        /* BlueTooth  stack enabled */
        case BTM_ENABLED_EVT:
            wiced_hal_pwm_configure_pin (WICED_GPIO_PIN_LED_1 ,PWM_CHANNEL);
            wiced_hal_pwm_start(PWM_CHANNEL, LHL_CLK, pwmToggle, pwmInit, 0);

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
	if(pwmToggle == PWM_MAX) /* Reset to 0% duty cycle once we reach 100% */
	{
		pwmToggle = PWM_INIT;
	}
	wiced_hal_pwm_change_values (PWM_CHANNEL, pwmToggle, pwmInit);
}

