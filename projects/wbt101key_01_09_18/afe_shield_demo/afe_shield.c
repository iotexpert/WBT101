/*
 * Copyright 2014, Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

 /** @file
 *
 * WICED sample application for SPI Master usage
 *
 * This application demonstrates how to use SPI driver interface
 * to send and receive bytes or a stream of bytes over the SPI hardware as a master.
 *
 * Features demonstrated
 * - SPI WICED APIs
 *
 * Application Instructions
 *
 * Usage
 */

#include "afe_shield_gatt_db.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "sparcommon.h"
#include "wiced_hal_puart.h"
//#include "wiced_hal_pspi.h"
#include "wiced_hal_i2c.h"
//GJL #include "wiced_hal_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "afe_shield_driver.h"
#include "wiced_led_manager.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
 static wiced_result_t  afe_shield_app_management_cback( wiced_bt_management_evt_t event,
                                                           wiced_bt_management_evt_data_t *p_event_data);
 static uint8_t afe_shield_write_8_bit_reg(uint8_t reg, int8_t value);

 static void afe_shield_app_init(void);

 static void seconds_timer_temperature_cb(uint32_t arg);

 static void afe_shield_set_advertisement_data(void);

 static wiced_bt_gatt_status_t afe_shield_get_value(uint16_t attr_handle,
         uint16_t conn_id, uint8_t *p_val, uint16_t len, uint16_t *p_len);

 static wiced_bt_gatt_status_t afe_shield_set_value(uint16_t attr_handle,
         uint16_t conn_id, uint8_t *p_val, uint16_t len);

 extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;    /* Manages runtime configuration of Bluetooth stack*/

 extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[]; /* Buffer for RF, HCI, ACL packets */

 uint16_t afe_shield_conn_id = 0;                                /* Manages connection IDs of client devices*/

/* *******************************************************************
*                              GATT Registration Callbacks
* *******************************************************************/
 static wiced_bt_gatt_status_t afe_shield_write_handler(
         wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id);

 static wiced_bt_gatt_status_t afe_shield_read_handler(
         wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id);

 static wiced_bt_gatt_status_t afe_shield_connect_callback(
         wiced_bt_gatt_connection_status_t *p_conn_status);

 static wiced_bt_gatt_status_t afe_shield_server_callback(uint16_t conn_id,
         wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data);

 static wiced_bt_gatt_status_t afe_shield_event_handler(
         wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START( )
{
//    wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
//    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    wiced_bt_stack_init( afe_shield_app_management_cback, &wiced_app_cfg_settings,
            wiced_app_cfg_buf_pools);
}

wiced_result_t afe_shield_app_management_cback( wiced_bt_management_evt_t event,
                                                  wiced_bt_management_evt_data_t *p_event_data)
{

    wiced_bt_dev_status_t status = WICED_NOTUP;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    WICED_BT_TRACE("Received Event : %d\n\n\r", event);

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            afe_shield_app_init();
            break;

        case BTM_DISABLED_EVT:
            /* Bluetooth Controller and Host Stack Disabled */
            WICED_BT_TRACE("Bluetooth Disabled\r\n");
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE("Advertisement State Change: %d\r\n", *p_adv_mode);
            if (BTM_BLE_ADVERT_OFF == *p_adv_mode) {
                /*Advertisements stopped*/
                wiced_result_t result;
                result = wiced_bt_start_advertisements(
                        BTM_BLE_ADVERT_UNDIRECTED_LOW, 0,
                        NULL);
                WICED_BT_TRACE("wiced_bt_start_advertisements: %d\r\n", result);
            }
            break;

        default:
            WICED_BT_TRACE(("Unhandled Bluetooth Management Event: %d\r\n", event));
            break;
        }

        return status;
}

/*
 Function Description:
 @brief    This function is executed if BTM_ENABLED_EVT event occurs  in afe_shield management callback.

 @param    void

 @return    void
 */

static void afe_shield_app_init(void) {
    /* Initialize Application */
    wiced_bt_app_init();

    /*AFE shield Initialization*/
    afe_shield_init();

    /* Starting the app 5 second timer */
    wiced_bt_app_start_timer(5, NULL, seconds_timer_temperature_cb, NULL);

    /* Set Advertisement Data */
    afe_shield_set_advertisement_data();

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register(afe_shield_event_handler);

    /* Initialize GATT Database */
    wiced_bt_gatt_db_init(gatt_database, gatt_database_len);

    /* Do not allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_FALSE, FALSE);

    /* Start Undirected LE Advertisements on device startup.*/
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);

}

static void seconds_timer_temperature_cb(uint32_t arg) {
    float temperature_float;
    int16_t temperature16;

    float humidity_float;
    int16_t humidity16;

    /* Hint: check that connection is up and client is registered to receive notifications for the below block of code */

    afe_shield_get_temperature_value((uint8_t *)&temperature_float);
    temperature16 = (int16_t)(temperature_float * 100);
    WICED_BT_TRACE("Temperature: %d.%d\n", (temperature16/100), (temperature16%100));

    afe_shield_get_humidity_value((uint8_t *)&humidity_float);
    humidity16 = (int16_t)(humidity_float * 100);
    WICED_BT_TRACE("Humidity: %d.%d\n", (humidity16/100), (humidity16%100));

    WICED_BT_TRACE("Temperature in hex: %04x\n", temperature16);
    if ((afe_shield_conn_id != 0)
            && (afe_shield_client_configuration[0] & GATT_CLIENT_CONFIG_NOTIFICATION != 0)) {

        afe_shield_last_temp_reading[0] = (uint8_t) (temperature16 & 0xff);
        afe_shield_last_temp_reading[1] = (uint8_t) ((temperature16 >> 8) & 0xff);

        afe_shield_last_hum_reading[0] = (uint8_t) (humidity16 & 0xff);
        afe_shield_last_hum_reading[1] = (uint8_t) ((humidity16 >> 8) & 0xff);

        WICED_BT_TRACE("Device afe_shield_conn_id:%d \r\n", afe_shield_conn_id);

        wiced_bt_gatt_send_notification(afe_shield_conn_id,
        HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,
                sizeof(afe_shield_last_temp_reading), afe_shield_last_temp_reading);

        wiced_bt_gatt_send_notification(afe_shield_conn_id,
        HDLC_ENVIRONMENTAL_SENSING_HUMIDITY_VALUE,
                sizeof(afe_shield_last_hum_reading), afe_shield_last_hum_reading);

    } else {
        WICED_BT_TRACE("Connection is not up\r\n");
    }
}

/*
 Function Name:
 afe_shield_set_advertisement_data

 Function Description:
 @brief  Set Advertisement Data

 @param void

 @return void
 */
static void afe_shield_set_advertisement_data(void) {

    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG
            | BTM_BLE_BREDR_NOT_SUPPORTED;


    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = sizeof(BIT16_TO_8(APPEARANCE_GENERIC_THERMOMETER));
    adv_elem[num_elem].p_data = (uint8_t *)BIT16_TO_8(APPEARANCE_GENERIC_THERMOMETER);
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len = sizeof(BIT16_TO_8(UUID_SERVICE_ENVIRONMENTAL_SENSING));
    adv_elem[num_elem].p_data = (uint8_t *)BIT16_TO_8(UUID_SERVICE_ENVIRONMENTAL_SENSING);
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen(
            (const char *) wiced_app_cfg_settings.device_name);
    adv_elem[num_elem].p_data = (uint8_t *) wiced_app_cfg_settings.device_name;
    num_elem++;

    if(WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem)){
        WICED_BT_TRACE("Raw advertisement failed\r\n");
    }
}

/*
Function Name:
 afe_shield_event_handler

 Function Description:
 @brief  This Function handles the GATT connection events - GATT Event Handler

 @param event            BLE GATT event type
 @param p_event_data     Pointer to BLE GATT event data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_event_handler(
        wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch (event) {
    case GATT_CONNECTION_STATUS_EVT:
        status = afe_shield_connect_callback(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = afe_shield_server_callback(p_attr_req->conn_id,
                p_attr_req->request_type, &p_attr_req->data);
        break;

    default:
        WICED_BT_TRACE("Other than GATT_CONNECTION_STATUS_EVT and GATT_ATTRIBUTE_REQUEST_EVT\r\n");
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/*
 Function Name:
 afe_shield_connect_callback

 Function Description:
 @brief  The callback function is invoked when GATT_CONNECTION_STATUS_EVT occurs in GATT Event handler function

 @param p_conn_status     Pointer to BLE GATT connection status

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_connect_callback(
        wiced_bt_gatt_connection_status_t *p_conn_status) {
    wiced_result_t gatt_status = WICED_BT_GATT_ERROR;

    if (p_conn_status->connected) {
        /* Device has connected */
        WICED_BT_TRACE("Connected : BDA '%B', Connection ID '%d'\n", p_conn_status->bd_addr, p_conn_status->conn_id);
//        wiced_hal_gpio_set_pin_output(WICED_P26, GPIO_PIN_OUTPUT_LOW);
//GJL        wiced_led_manager_enable_led(WICED_PLATFORM_LED_1);
        afe_shield_conn_id = p_conn_status->conn_id;
        gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC, NULL);
    } else {
        /* Device has disconnected */
        WICED_BT_TRACE("Disconnected : BDA '%B', Connection ID '%d', Reason '%d'\n", p_conn_status->bd_addr, p_conn_status->conn_id, p_conn_status->reason);
        afe_shield_conn_id = 0;
//        wiced_hal_gpio_set_pin_output(WICED_P26, GPIO_PIN_OUTPUT_HIGH);
//GJL        wiced_led_manager_disable_led(WICED_PLATFORM_LED_1);
        gatt_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);
    }

    return gatt_status;
}

/*
 Function Name:
 afe_shield_server_callback

 Function Description:
 @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs in GATT Event handler function.
 GATT Server Event Callback function.

 @param conn_id  Connection ID from GATT Connection event
 @param type     GATT Request type
 @param p_data   Pointer to BLE GATT request data

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_server_callback(uint16_t conn_id,
        wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data) {
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch (type) {
    case GATTS_REQ_TYPE_READ:
        status = afe_shield_read_handler(&p_data->read_req, conn_id);
        break;

    case GATTS_REQ_TYPE_WRITE:
        status = afe_shield_write_handler(&p_data->write_req, conn_id);
        break;
    }

    return status;
}

/*
 Function Name:
 afe_shield_write_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the client device and is invoked by GATT Server Event Callback function. This handles "Write Requests" received from Client device

 @param p_write_req   Pointer to BLE GATT write request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_write_handler(
        wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id) {
    /* Attempt to perform the Write Request */
    return afe_shield_set_value(p_write_req->handle, conn_id,
            p_write_req->p_val, p_write_req->val_len);
}

/*
 Function Name:
 afe_shield_read_handler

 Function Description:
 @brief  The function is invoked when GATTS_REQ_TYPE_READ is received from the client device
 and is invoked by GATT Server Event Callback function. This handles "Read Requests" received from Client device

 @param p_write_req   Pointer to BLE GATT read request
 @param conn_id  Connection ID from GATT Connection event

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_read_handler(
        wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id) {
    /* Attempt to perform the Read Request */
    return afe_shield_get_value(p_read_req->handle, conn_id, p_read_req->p_val,
            *p_read_req->p_val_len, p_read_req->p_val_len);
}

/*
 Function Name:
 afe_shield_get_value

 Function Description:
 @brief  The function is invoked by afe_shield_read_handler to get a Value from GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT read request value
 @param len      Maximum length of GATT read request
 @param p_len        Pointer to BLE GATT read request length

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_get_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len, uint16_t *p_len) {
    int i = 0;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < afe_shield_gatt_db_ext_attr_tbl_size; i++) {
        if (afe_shield_gatt_db_ext_attr_tbl[i].handle == attr_handle) {
            /* Detected a matching handle in the external lookup table */
            if (afe_shield_gatt_db_ext_attr_tbl[i].offset <= len) {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = afe_shield_gatt_db_ext_attr_tbl[i].offset;
                memcpy(p_val, afe_shield_gatt_db_ext_attr_tbl[i].p_data,
                        afe_shield_gatt_db_ext_attr_tbl[i].offset);
                res = WICED_BT_GATT_SUCCESS;
            } else {
                /* Value to read will not fit within the buffer */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    return res;
}

/*
 Function Name:
 afe_shield_set_value

 Function Description:
 @brief  The function is invoked by afe_shield_write_handler to set a Value to GATT DB.

 @param attr_handle  GATT attribute handle
 @param conn_id      Connection ID from GATT Connection event
 @param p_val        Pointer to BLE GATT write request value
 @param len          length of GATT write request

 @return wiced_bt_gatt_status_t  BLE GATT status
 */
static wiced_bt_gatt_status_t afe_shield_set_value(uint16_t attr_handle,
        uint16_t conn_id, uint8_t *p_val, uint16_t len) {
    int i = 0;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < afe_shield_gatt_db_ext_attr_tbl_size; i++) {
        if (afe_shield_gatt_db_ext_attr_tbl[i].handle == attr_handle) {
            /* Verify that size constraints have been met */
            if (afe_shield_gatt_db_ext_attr_tbl[i].len >= len) {
                /* Value fits within the supplied buffer; copy over the value */
                afe_shield_gatt_db_ext_attr_tbl[i].offset = len;
                memcpy(afe_shield_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;
            } else {
                /* Value to write does not meet size constraints */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    return res;
}
