/* Display "Hello World!" on the OLED */


// GJL: Edits to get library to work:
// 1. Copy 43xxx_Wi-Fi/libraries/graphics/u8g to the project directory
// 2. Add #include "u8g/u8h_arm.h" to the project c file
// 3. Add all of the u8g C source files to the makefile.mk for the project
// 4. Change u8g_init_wiced_i2c_device(wiced_i2c_device_t* device); in header and C file to use correct I2C structure
// 5. Change i2c_display structure to i2c_pins and i2c_address variables in u8g_arm.c
// 6. Change all the I2C Write functions in u8g_arm.c
// 7. Remove probe device from u8g_arm.c
#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_i2c.h"
#include "u8g/u8g_arm.h"

/*****************************    Constants   *****************************/
#define I2C_ADDRESS  (0x3C <<1) /* The I2C address is 8 bit so need to left shift the 7-bit address by 1 */

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

uint8 I2Cdata[2];

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

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
    static u8g_t u8g;
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth  stack enabled */
        case BTM_ENABLED_EVT:
        	/* Initializes the GPIO driver and enable GPIO (LHL) interrupts */
            wiced_hal_gpio_init( );

            /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
            wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

            /* Configure I2C block and pins for the display */
            u8g_init_wiced_i2c_device(WICED_I2C_SDA_I2S_DOUT_PCM_OUT_SCL_I2S_DIN_PCM_IN, I2C_ADDRESS);

            /* Write to the display */
            u8g_FirstPage(&u8g);
            do {
                    //u8g_SetFont(&u8g, u8g_font_unifont);
                    u8g_SetFontPosTop(&u8g);
                    u8g_DrawStr(&u8g, 0, 10, "Hello World!");
            } while (u8g_NextPage(&u8g));

            break;
        default:
            break;
    }
    return result;
}

