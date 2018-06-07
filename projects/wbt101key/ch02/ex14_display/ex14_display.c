/* Display Text and Graphics on the OLED */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "u8g_arm.h"

/*****************************    Constants   *****************************/
#define THREAD_DELAY_IN_MS          (250)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/* I2C address of the display */
#define I2C_ADDRESS (0x3C)

/*****************************    Variables   *****************************/
wiced_thread_t * display_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void display( uint32_t arg );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL );                    // Register BT stack callback
}


/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:
            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

            WICED_BT_TRACE("GJL: Start\n");


            /* Start a thread to control the OLED display */
            display_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    display_thread,                 // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Display",                      // Name
                    display,                        // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to control the OLED display */
void display( uint32_t arg )
{
    u8g_t u8g;
    uint8_t opt1 = 0x01;
    uint8_t opt2 = 0x08;

    u8g_init_wiced_i2c_device( I2C_ADDRESS );
    u8g_InitComFn( &u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn );

    for(;;)
    {
        u8g_FirstPage( &u8g );
        do
        {
            u8g_SetFont( &u8g, u8g_font_unifont );
            u8g_SetFontPosTop( &u8g );
            u8g_DrawStr( &u8g, 0, 0, "Unifont!" );

            u8g_SetFont( &u8g, u8g_font_fur11 );
            u8g_SetFontPosTop( &u8g );
            u8g_DrawStr( &u8g, 0, 20, "Free Universal!" );

            u8g_SetFont( &u8g, u8g_font_gdb20 );
            u8g_SetFontPosTop( &u8g );
            u8g_DrawStr( &u8g, 0, 35, "Gentium!" );

            /* Arguments to DrawCirle are: displayName, XPosition, YPosition, Radius, Option */
            /* Option specifies the quadrant to draw: 0x01=topRight, 0x02=topLeft, 0x04=botLeft, 0x08=botRight, 0x0F=fullCircle */
            u8g_DrawCircle (&u8g, 110, 10, 6, opt1 ); /* Draw 1/4 of circle 1 */
            u8g_DrawCircle( &u8g, 120, 10, 8, opt2 ); /* Draw 1/4 of circle 2 */
        } while( u8g_NextPage( &u8g ) );

        /* Shift left so that the next quarter of circle 1 is drawn on the next pass (anti-clockwise) */
        opt1 <<= 1;
        if( opt1 == 0x10 )
        {
            opt1 = 0x01;
        }

        /* Shift right so that the next quarter of circle 1 is drawn on the next pass (anti-clockwise) */
        opt2 >>= 1;
        if( opt2 == 0 )
        {
            opt2 = 0x08;
        }

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
