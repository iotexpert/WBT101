/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor
 *  Corporation. All rights reserved. This software, including source code, documentation and  related
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit
 * products. Any reproduction, modification, translation, compilation,  or representation of this
 * Software except as specified above is prohibited without the express written permission of
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to
 * the Software without notice. Cypress does not assume any liability arising out of the application
 * or use of the Software or any product or circuit  described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or failure of the
 * Cypress product may reasonably be expected to result  in significant property damage, injury
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees
 * to indemnify Cypress against all liability.
 */

/** @file
*
* BLE Vendor Specific Device
*
* This file provides definitions and function prototypes for Motion Sensor
* device
*
*/

#ifndef _ex07_low_power_BLE_H_
#define _ex07_low_power_BLE_H_

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define MAX_SECURITY_KEY_SIZE           0x10

#define ex07_low_power_INTERRUPT WICED_P38 /* Motion Sensor interrupt on P38 */

#define RTC_OSC_FREQ           32 /* 32KHz frequency for RTC oscillator */

#define NOTFICATION_TIME_MS    500 /* 500 ms interval for notification */
#define IDLE_TIME_S            10 /* 10 s interval for idle timeout */

/* VSIDs for NVRAM storage */
#define ex07_low_power_VS_ID                      WICED_NVRAM_VSID_START

#define ENABLE_GPIO_INTERRUPT wiced_hal_gpio_configure_pin(ex07_low_power_INTERRUPT,(GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_EN_INT_RISING_EDGE), GPIO_PIN_OUTPUT_LOW)
#define DISABLE_GPIO_INTERRUPT wiced_hal_gpio_configure_pin(ex07_low_power_INTERRUPT,(GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_INTERRUPT_DISABLE), GPIO_PIN_OUTPUT_LOW)

#define NO_ACTION 0
#define GET_PASSKEY 6
/******************************************************************************
*                             Function prototypes
******************************************************************************/
wiced_result_t ex07_low_power_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void ex07_low_power_application_init(void);

uint32_t ex07_low_power_sleep_handler(wiced_sleep_poll_type_t type );

void ex07_low_power_passkey_reply (uint32_t passkey);

void ex07_low_power_smp_bond_result( uint8_t result );

wiced_bt_gatt_status_t ex07_low_power_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);

void ex07_low_power_set_advertisement_data(void);

wiced_bt_gatt_status_t ex07_low_power_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );

wiced_bt_gatt_status_t ex07_low_power_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status );

wiced_bt_gatt_status_t ex07_low_power_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status );

wiced_bt_gatt_status_t ex07_low_power_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );

wiced_bt_gatt_data_t * ex07_low_power_get_attribute( uint16_t handle );

wiced_bt_gatt_status_t ex07_low_power_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );

wiced_bt_gatt_status_t ex07_low_power_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );

wiced_bt_gatt_status_t ex07_low_power_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu);

void send_sensor_value_notification();

void ex07_low_power_interrupt_handler(void* user_data, uint8_t value );

void notification_timer_callback(uint32_t arg);

void idle_timer_callback(uint32_t arg);

void remove_bond_data_and_start_advertisement(void);

void button_cb (void* user_data, uint8_t value );

void shutdown_cb(void);

void ex07_low_power_set_console_input( void );

static void ex07_low_power_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );

void send_battery_notification(void);

void shutdown_cb(void);

#endif /* _ex07_low_power_BLE_H_ */
