/*
 * Copyright Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

 /** @file
 *
 * Description:
 * This example projects interfaces with the STMicroelectronics LSM9DS1
 * 9-axis motion sensor and sends the data over a BLE connection
 * while maintaining extremely low power consumption for coin cell
 * usage. It acts as a GATT server and maintains bond information
 * for 1 Central device. It also monitors the battery level and
 * sends battery details to the Client device.
 *
 * Hardware:
 * CYW920719Q40EVB-01/CYW920735Q60EVB-01 Evaluation Board is used to develop this firmware.
 * The motion sensor is on the board and connected to the CYW20719/CYW20735 device
 * via I2C. The only external connection required is
 * from test point TP1 (near motion sensor IC U2) to D13 (P38) for the CYW920719Q40EVB-01
 * board and J17.1 to D10 (P38) for the CYW920735Q60EVB-01 board
 *
 * Testing:
 * This example project can be tested using CySmart software and a BLE dongle.
 * The following steps will enable you to test the basic functionality:
 *
 * 1. Connect a wire from TP1 to D13/D10
 * 2. Power on the CYW920719Q40EVB-01/CYW920735Q60EVB-01 kit
 * 3. Open a Serial port emulator like TeraTerm. You will be able to see all the logs.
 * 4. Using CySmart, scan for advertising devices
 * 5. Connect to the device named ‘Low Power’
 * 6. Click on Pair. After pairing the device will be in SDS mode.
 * 7. Click on Discover Attributes. You will be able to see the GATT database
 * 8. Click on Enable notifications. The device will send notifications and go back to sleep.
 * 9. Move the evaluation board a little. This will wake up the device and send notifications.
 * 10. Click on Disconnect in CySmart. The device will start directed advertising.
 * 11. Press the button SW3 to erase bond information and start undirected advertisement
 *
 * This file initializes the BLE stack and initializes PUART for Debug
 *
 */

#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_cfg.h"
#include "wiced_sleep.h"
#include "wiced_platform.h"
#include "wiced_transport.h"


/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

extern uint32_t ex07_low_power_sleep_handler(wiced_sleep_poll_type_t type );
extern wiced_result_t ex07_low_power_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

extern uint8_t                 allow_sleep;              /* Store Whether sleep allowed or not */
/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define ex07_low_power_INTERRUPT WICED_P38
/******************************************************************************
 *                                Structures
 ******************************************************************************/
/* transport configuration */
const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, 115200},
    { 0, 0},
    NULL,
    NULL,
    NULL
};
/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
wiced_sleep_config_t    ex07_low_power_sleep_config; /*sleep configuration*/

uint8_t                 ex07_low_power_boot_mode; /* Store Boot mode: Cold or Fast */
 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/******************************************************************************/
/*
 *  Entry point to the application.
 *****************************************************************************/
APPLICATION_START( )
{

    wiced_transport_init( &transport_cfg );

    /* Route Trace messages to PUART */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    /* Register call back and configuration with stack */
     if(WICED_BT_SUCCESS != wiced_bt_stack_init( ex07_low_power_management_cback ,
                     &wiced_app_cfg_settings, wiced_app_cfg_buf_pools ))
     {
         WICED_BT_TRACE("Stack Init failed\n\r");
     }

     allow_sleep = 1;

     /* configure to sleep if sensor is idle */
     ex07_low_power_sleep_config.sleep_mode             = WICED_SLEEP_MODE_NO_TRANSPORT;
     ex07_low_power_sleep_config.device_wake_mode       = WICED_SLEEP_WAKE_ACTIVE_LOW;
     ex07_low_power_sleep_config.device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO;
     ex07_low_power_sleep_config.device_wake_gpio_num   = ex07_low_power_INTERRUPT;
     ex07_low_power_sleep_config.host_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_HIGH;
     ex07_low_power_sleep_config.sleep_permit_handler   = ex07_low_power_sleep_handler;

     if(WICED_BT_SUCCESS != wiced_sleep_configure(&ex07_low_power_sleep_config))
     {
         WICED_BT_TRACE("Sleep Configure failed\n\r");
     }

     /* Check if Cold Boot or Fast Boot */
     ex07_low_power_boot_mode = wiced_sleep_get_boot_mode();
}
