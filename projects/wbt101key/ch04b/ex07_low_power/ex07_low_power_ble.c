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
 * This file has the BLE functionality
 *
 */

#include "ex07_low_power_gatt_db.h"
#include "wiced_sleep.h"
#include "ex07_low_power_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "rtc.h"
#include "wiced_bt_fw_upgrade.h"
#include "wiced_bt_l2c.h"
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;

extern uint8_t          ex07_low_power_boot_mode; /* Store Boot mode: Cold or Fast */

extern Point    ecdsa256_public_key;

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
uint8_t                 enable_sds = 0;              /* Store Whether SDS allowed or not */
uint8_t                 allow_sleep;              /* Store Whether sleep allowed or not */

wiced_timer_t           notification_timer_handle, idle_timer_handle; /* Notification timer and Idle timer handles */

uint8_t user_input_state;

uint32_t passkey = 0;
uint8_t passkey_itr = 0;

uint8_t nv_update = WICED_FALSE; /* Global structure to check if data needs to be written to NVRAM */

/******************************************************************************
*                                Function Definitions
******************************************************************************/
/**
 * Function         ex07_low_power_management_cback
 *
 *                  This function is invoked after the controller's initialization is complete
 *
 * @param[in] event                : Callback Event number
 * @param[in] p_event_data         : Event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_result_t ex07_low_power_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;

    WICED_BT_TRACE("Motion Sensor management cback: %d\n\r", event );

    switch( event )
    {
        case BTM_ENABLED_EVT:
            /* Initialize the application */
            ex07_low_power_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("numeric_value: %d \r\n\r", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \r\n\r", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PASSKEY_REQUEST_EVT:
            allow_sleep = 0;
            WICED_BT_TRACE("PassKey requested\r\n\r", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            user_input_state = GET_PASSKEY;
            WICED_BT_TRACE ("Enter Six Digit Passkey\r\n\r");
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Grant security */
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap   = BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data       = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req       = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size   = MAX_SECURITY_KEY_SIZE;
            p_event_data->pairing_io_capabilities_ble_request.init_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d\n\r", p_info->reason);
            /* Process SMP bond result */
            ex07_low_power_smp_bond_result( p_info->status );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            ex07_low_power_hostinfo.link_keys = p_event_data->paired_device_link_keys_update;
            nv_update = WICED_TRUE;
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            wiced_hal_read_nvram( ex07_low_power_VS_ID, sizeof(ex07_low_power_hostinfo), (uint8_t*)&ex07_low_power_hostinfo, &result );
            p_event_data->paired_device_link_keys_request = ex07_low_power_hostinfo.link_keys;
            WICED_BT_TRACE("keys read from NVRAM %B result: %d \n\r", (uint8_t*)&ex07_low_power_hostinfo.link_keys, result);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            ex07_low_power_hostinfo.local_keys = p_event_data->local_identity_keys_update;
            nv_update = WICED_TRUE;
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            wiced_hal_read_nvram( ex07_low_power_VS_ID, sizeof(ex07_low_power_hostinfo), (uint8_t*)&ex07_low_power_hostinfo, &result );
            WICED_BT_TRACE("Host info read from NVRAM result: %d \n\r",  result);
            /* read keys from NVRAM */
            if (1 == ex07_low_power_hostinfo.dev_prebonded)
            {
                p_event_data->local_identity_keys_request = ex07_low_power_hostinfo.local_keys;
                WICED_BT_TRACE("local keys read from NVRAM result: %d \n\r",  result);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Device not bonded\n\r");
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d\n\r", p_status->bd_addr, p_status->result);
            /*Allow shutdown only after encryption as encryption can't be done in SDS */
            if(WICED_SUCCESS == p_status->result)
            {
                enable_sds = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
                allow_sleep = 1;
            }
        break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n\r", *p_mode);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE(" BTM_BLE_CONNECTION_PARAM_UPDATE status:%d, Connection Interval: %d, Connection Latency: %d, Connection Timeout: %d\n\r", p_event_data->ble_connection_param_update.status, p_event_data->ble_connection_param_update.conn_interval, p_event_data->ble_connection_param_update.conn_latency, p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        default:
            WICED_BT_TRACE(("Unhandled Bluetooth Management Event: %d\r\n\r", event));
            break;
    }

    return result;
}

/**
 * Function         ex07_low_power_application_init
 *
 * @brief           This function is invoked after ENABLED event from controller
 *
 * @return    None
 */
void ex07_low_power_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result, timer_result;

    WICED_BT_TRACE( "\n\r\r--------------------------------------------------------------- \r\n\r\n\r"
                             "                           Motion Sensor                      \r\n\r\n\r"
                             "---------------------------------------------------------------\n\r\r");

    if(WICED_SLEEP_COLD_BOOT ==  ex07_low_power_boot_mode)
    {
        WICED_BT_TRACE( "Motion Sensor cold Start \n\r");
    }
    else
    {
        WICED_BT_TRACE( "Motion Sensor warm Start \n\r");
    }

    if (!wiced_ota_fw_upgrade_init(&ecdsa256_public_key, NULL))
    {
        WICED_BT_TRACE("OTA upgrade Init failure!!!\n\r");
    }

    /* Initialize PUART for input */
    ex07_low_power_set_console_input();

    /* Initialize I2C */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* Register GPIO for interrupt and button */
    wiced_hal_gpio_register_pin_for_interrupt( ex07_low_power_INTERRUPT, ex07_low_power_interrupt_handler, NULL );
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, button_cb, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* Initialize battery monitor */
    enable_battmon();

    /* Initialize Idle timer and Notification timer */
    if(WICED_BT_SUCCESS != wiced_init_timer(&notification_timer_handle, &notification_timer_callback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Notification timer failed\n\r");
    }
    if(WICED_BT_SUCCESS != wiced_init_timer(&idle_timer_handle, &idle_timer_callback, 0, WICED_SECONDS_TIMER))
    {
        WICED_BT_TRACE("Idle timer failed\n\r");
    }

    wiced_bt_dev_register_hci_trace( ex07_low_power_hci_trace_cback );

    /* Read all sensors and update GATT DB */
    read_all_sensors_update_gatt();

    /* Read data from NVRAM in case data is available */
    wiced_hal_read_nvram( ex07_low_power_VS_ID, sizeof(ex07_low_power_hostinfo), (uint8_t*)&ex07_low_power_hostinfo, &result );
    WICED_BT_TRACE("Host info read from NVRAM result: %d \n\r",  result);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(ex07_low_power_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n\r", gatt_status);

    /*  Tell stack to use our GATT database */
    WICED_BT_TRACE("GATT DB length: %d\n\r", gatt_database_len);
    gatt_status =  wiced_bt_gatt_db_init( gatt_database, gatt_database_len );
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n\r", gatt_status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
    WICED_BT_TRACE("Allow Pairing\n\r");

    /* If cold boot then initialize GPIO, RTC and LSM9DS1 sensor */
    if(WICED_SLEEP_COLD_BOOT == ex07_low_power_boot_mode)
    {
        ENABLE_GPIO_INTERRUPT;

        rtcConfig.oscillatorFrequencykHz= RTC_OSC_FREQ;
        rtc_init();

        initialize_all_sensors();

        power_down_ex07_low_power();
    }

  /* If Device not currently connected and no bonded device do undirected advertisement */
   if(0 == ex07_low_power_state.conn_id && 0 == ex07_low_power_hostinfo.dev_prebonded)
  {
     /* Set the advertising params and make the device discoverable */
       enable_sds = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
       ex07_low_power_set_advertisement_data();
       result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
       WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n\r", result );
  }
 /* If Device not currently connected but bonded device present do directed advertisement */
   else if (0 == ex07_low_power_state.conn_id && 1 == ex07_low_power_hostinfo.dev_prebonded)
    {
        /* Set the advertising params and make the device discoverable */
        enable_sds = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
        WICED_BT_TRACE( "Address: %B, Address type: %d\n\r", ex07_low_power_hostinfo.bdaddr, ex07_low_power_hostinfo.address_type);
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_DIRECTED_LOW, ex07_low_power_hostinfo.address_type,  ex07_low_power_hostinfo.bdaddr);
        WICED_BT_TRACE( "wiced_bt_start_advertisements Directed %d\n\r", result );
    }
    else
    {
        enable_sds = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
    }
}

/**
 * Function         ex07_low_power_sleep_handler
 *
 * @brief           This function is invoked by the controller for sleep request
 *
 * @param[in] type                : Type of return required
 *
 * @return    WICED_SLEEP_ALLOWED_WITH_SHUTDOWN/WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN: if parameter WICED_SLEEP_POLL_SLEEP_PERMISSION
 *            WICED_SLEEP_MAX_TIME_TO_SLEEP : if parameter WICED_SLEEP_POLL_TIME_TO_SLEEP
 */
uint32_t ex07_low_power_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    update_battery_level_and_report();

    if ( WICED_TRUE == nv_update )
    {
        wiced_result_t rc;
        int bytes_written = wiced_hal_write_nvram( ex07_low_power_VS_ID, sizeof(ex07_low_power_hostinfo), (uint8_t*)&ex07_low_power_hostinfo, &rc );
        WICED_BT_TRACE("NVRAM write:%d rc:%d", bytes_written, rc);
        nv_update = WICED_FALSE;
   }


    switch(type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            if(1 == allow_sleep)
            {
                if(WICED_SLEEP_ALLOWED_WITH_SHUTDOWN == enable_sds)
                {
                    ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
                    WICED_BT_TRACE(".");
                }
                else
                {
                    ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
                    WICED_BT_TRACE("$");
                }
            }
            break;
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if(1 == allow_sleep)
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            else
            {
                ret = 0;
            }
            break;
    }
    return ret;
}

/**
 * Function         ex07_low_power_gatts_callback
 *
 * @brief           This function is invoked on GATT event
 *
 * @param[in] event                : Gatt event
 * @param[in] p_data               : Gatt event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("GATT EVT: %d\n\r", event);
    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = ex07_low_power_gatts_conn_status_cb( &p_data->connection_status );
        if(WICED_BT_GATT_SUCCESS != result)
        {
            WICED_BT_TRACE("GATT Connection CB failed: %d", result);
        }
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = ex07_low_power_gatts_req_cb( &p_data->attribute_request );
        if(WICED_BT_GATT_SUCCESS != result)
        {
            WICED_BT_TRACE("GATT Req CB failed: %d", result);
        }
        break;

    default:
        break;
    }
    return result;
}

/**
 * Function         ex07_low_power_gatts_conn_status_cb
 *
 * @brief           This function is invoked on GATT connection event
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : Connection status from connection up/down function
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    /* Pass connection up/down event to the OTA FW upgrade library */
    wiced_ota_fw_upgrade_connection_status_event(p_status);

    if ( p_status->connected )
    {
        return ex07_low_power_gatts_connection_up( p_status );
    }

    return ex07_low_power_gatts_connection_down( p_status );
}

/**
 * Function         ex07_low_power_gatts_connection_up
 *
 * @brief           This function is invoked when connection is established
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "motion sensor conn up %B id:%d\n\r:", p_status->bd_addr, p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    ex07_low_power_state.conn_id = p_status->conn_id;
    memcpy(ex07_low_power_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    WICED_BT_TRACE( "Stopping Advertisements%d\n\r", result );

    /* Saving host info in NVRAM  */
    memcpy( ex07_low_power_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    ex07_low_power_hostinfo.address_type = p_status->addr_type;

    uint8_t bytes_written = 0;
    nv_update = WICED_TRUE;
    WICED_SUPPRESS_WARNINGS(bytes_written);

    /* Update connection parameters to 100 ms for SDS */
    wiced_bt_l2cap_update_ble_conn_params( p_status->bd_addr, 80, 80, 0, 512 );

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         ex07_low_power_gatts_connection_down
 *
 * @brief           This function is invoked when connection is lost
 *
 * @param[in] p_status               : Gatt connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n\r", ex07_low_power_state.remote_addr, p_status->conn_id, p_status->reason );

    /* Resetting the device info */
    memset( ex07_low_power_state.remote_addr, 0, 6 );
    ex07_low_power_state.conn_id = 0;
    enable_sds = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

    /* If device bonded then do directed advertisement */
    if(1 == ex07_low_power_hostinfo.dev_prebonded)
    {
        /* Set the advertising params and make the device discoverable */
        WICED_BT_TRACE( "Address: %B, Address type: %d\n\r", ex07_low_power_hostinfo.bdaddr, ex07_low_power_hostinfo.address_type);
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_DIRECTED_LOW, ex07_low_power_hostinfo.address_type, ex07_low_power_hostinfo.bdaddr);
        WICED_BT_TRACE( "wiced_bt_start_advertisements Directed %d\n\r", result );
    }
    /* If device not bonded then do undirected advertisement */
    else
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n\r", result );
    }

    WICED_SUPPRESS_WARNINGS(result);

    return WICED_BT_SUCCESS;
}

/**
 * Function         ex07_low_power_gatts_req_cb
 *
 * @brief           Process GATT request from the peer
 *
 * @param[in] p_data                 : Gatt attribute request data
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE( "wiced_gatts_req_cb. conn %d, type %d\n\r", p_data->conn_id, p_data->request_type );

    /* Check the type of request and service it by function calls */
    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = ex07_low_power_gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = ex07_low_power_gatts_req_write_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_CONF:
        /* if indication confirmation is for the OTA FW upgrade service, pass it to the library to process */
        if (p_data->data.handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
        {
            result = wiced_ota_fw_upgrade_indication_cfm_handler(p_data->conn_id, p_data->data.handle);
        }

    case GATTS_REQ_TYPE_MTU:
        result = ex07_low_power_gatts_req_mtu_handler( p_data->conn_id, p_data->data.mtu );
        break;

   default:
        break;
    }
    return result;
}

/**
 * Function         ex07_low_power_get_attribute
 *
 * @brief           Find the right data from Gatt DB
 *
 * @param[in] handle                 : Gatt handle
 *
 * @return                           : Correct attribute pointer from Gatt DB
 */
wiced_bt_gatt_data_t * ex07_low_power_get_attribute( uint16_t handle )
            {
    int array_index;
    for ( array_index = 0; array_index < ex07_low_power_gatt_db_ext_attr_tbl_size; array_index++ )
    {
        if ( ex07_low_power_gatt_db_ext_attr_tbl[array_index].handle == handle )
        {
            return ( &ex07_low_power_gatt_db_ext_attr_tbl[array_index] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n\r", handle );
    return NULL;
}


/**
 * Function         ex07_low_power_gatts_req_read_handler
 *
 * @brief           Process Read request or command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from Gatt read request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    wiced_bt_gatt_data_t *puAttribute;
    int          attr_len_to_copy;

    /* if read request is for the OTA FW upgrade service, pass it to the library to process */
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }

    /* Get the right address for the handle in Gatt DB */
    if (NULL == ( puAttribute = ex07_low_power_get_attribute(p_read_data->handle) ))
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n\r", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n\r", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    /* If Sensor value requested then update the Gatt DB with latest value */
    if(HDLC_ex07_low_power_NOTIFY_VALUE == p_read_data->handle)
    {
        read_all_sensors_update_gatt();
    }

    /* If battery level requested then update the Gatt DB with latest value */
    if(HDLC_BATTERY_VALUE == p_read_data->handle)
    {
        update_battery_level_and_report();
    }

    if ( p_read_data->offset >= puAttribute->len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         ex07_low_power_gatts_req_write_handler
 *
 * @brief           Process write request or write command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from Gatt write request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;

    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n\r ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    /* if write request is for the OTA FW upgrade service, pass it to the library to process */
    if (p_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        if(battery_level < CRITICAL_LEVEL)
        {
            return WICED_BT_GATT_INTERNAL_ERROR;
        }
        else
        {
            return wiced_ota_fw_upgrade_write_handler(conn_id, p_data);
        }
    }

    switch ( p_data->handle )
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification or indication */
    case HDLD_ex07_low_power_CLIENT_CONFIGURATION:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        ex07_low_power_hostinfo.sensor_value_characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
        /* If notifications enabled then start notification and idle timer.
         * Send notifications at 500 ms interval and check for interrupt from
         * sensor after 10s. If interrupt not active then got to SDS
         */
        if(0 != ex07_low_power_hostinfo.sensor_value_characteristic_client_configuration)
        {
            ex07_low_power_acc_interrupt_enable();
            if(WICED_BT_SUCCESS != wiced_start_timer(&notification_timer_handle, NOTFICATION_TIME_MS))
            {
                WICED_BT_TRACE("Notification timer start failed\n\r");
            }
            if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, IDLE_TIME_S))
            {
                WICED_BT_TRACE("Idle timer start failed\n\r");
            }
        }
        else
        {
            ex07_low_power_acc_interrupt_disable();
        }
        nv_update = WICED_TRUE;
        break;

    case HDLD_ex07_low_power_BATTERY_CLIENT_CONFIGURATION:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }

        ex07_low_power_hostinfo.battery_characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
        nv_update = WICED_TRUE;
        if(0 != ex07_low_power_hostinfo.battery_characteristic_client_configuration);
        {
            /* Send battery value once */
            if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(ex07_low_power_state.conn_id, HDLC_BATTERY_VALUE, ex07_low_power_gatt_db_ext_attr_tbl[7].len, ex07_low_power_gatt_db_ext_attr_tbl[7].p_data))
            {
                WICED_BT_TRACE("Battert notification send failed\n\r");
            }
        }
        break;

        default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return  result;
}

void ex07_low_power_passkey_reply (uint32_t passkey)
{
   wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS, ex07_low_power_state.remote_addr ,passkey);
}

/**
 * Function         ex07_low_power_smp_bond_result
 *
 * @brief           Process SMP bonding result. If we successfully paired with the
 *                  central device, save its BDADDR in the NVRAM and initialize
 *                  associated data
 *
 * @param[in] result                : Bonding result
 *
 * @return                          : None
 */
void ex07_low_power_smp_bond_result( uint8_t result )
{
    uint16_t written_byte;
    wiced_result_t status;

    /* Bonding success */
    if ( WICED_BT_SUCCESS == result)
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( ex07_low_power_hostinfo.bdaddr, ex07_low_power_state.remote_addr, sizeof( BD_ADDR ) );
        ex07_low_power_hostinfo.dev_prebonded = 1;
        /* Write to NVRAM */
        nv_update = WICED_TRUE;
        enable_sds = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
    }

    WICED_SUPPRESS_WARNINGS(written_byte);
}

/**
 * Function         ex07_low_power_gatts_req_mtu_handler
 *
 * @brief           Process MTU request from the peer
 *
 * @return                          : None
 */
wiced_bt_gatt_status_t ex07_low_power_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n\r", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         ex07_low_power_set_advertisement_data
 *
 * @brief           Set Advertisement data
 *
 * @return                          : None
 */
void ex07_low_power_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t ex07_low_power_service_uuid[LEN_UUID_128] = { UUID_SERVICE_ex07_low_power };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = ex07_low_power_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (const char *)wiced_app_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )wiced_app_cfg_settings.device_name;
    num_elem++;

    if(WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Set advertisement data failed\n\r");
    }
}

/**
 * Function         send_sensor_value_notification
 *
 * @brief           Send sensor value notification if CCCD == 1 and device connected
 *
 * @return                          : None
 */

void send_sensor_value_notification()
{
    if((0 != ex07_low_power_hostinfo.sensor_value_characteristic_client_configuration) && (0 != ex07_low_power_state.conn_id));
    {
       WICED_BT_TRACE("Sending Notification\n\r");
       read_all_sensors_update_gatt();
       WICED_BT_TRACE("Device ex07_low_power_conn_id:%d \r\n\r", ex07_low_power_state.conn_id);
       if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(ex07_low_power_state.conn_id, HDLC_ex07_low_power_NOTIFY_VALUE, ex07_low_power_gatt_db_ext_attr_tbl[2].len, ex07_low_power_gatt_db_ext_attr_tbl[2].p_data))
       {
           WICED_BT_TRACE("Sending sensor value notification failed\n\r");
       }
    }
}

/**
 * Function         send_battery_notification
 *
 * @brief           Send battery value notification if CCCD == 1 and device connected
 *
 * @return                          : None
 */
void send_battery_notification(void)
{
    if(0 != ex07_low_power_hostinfo.battery_characteristic_client_configuration && 0 != ex07_low_power_state.conn_id);
    {
       WICED_BT_TRACE("Sending Battery Notification\n\r");
       WICED_BT_TRACE("Device ex07_low_power_conn_id:%d \r\n\r", ex07_low_power_state.conn_id);
       wiced_bt_gatt_send_notification(ex07_low_power_state.conn_id, HDLC_BATTERY_VALUE, ex07_low_power_gatt_db_ext_attr_tbl[7].len, ex07_low_power_gatt_db_ext_attr_tbl[7].p_data);
    }
}

/**
 * Function         ex07_low_power_interrupt_handler
 *
 * @brief           Interrupt handler
 * @param[in] user_data                : User data (not used)
 * @param[in] value                    : value (not used)
 *
 * @return                          : None
 */
void ex07_low_power_interrupt_handler(void* user_data, uint8_t value )
{
    WICED_BT_TRACE("Interrupt Handler\n\r");
    /* Disable the GPIO interrupt */
    DISABLE_GPIO_INTERRUPT;
    /* Check if device is still connected and CCCD==1. If not then disable the interrupt on the sensor */
    if(0 == ex07_low_power_hostinfo.sensor_value_characteristic_client_configuration || 0 == ex07_low_power_state.conn_id)
    {
        ex07_low_power_acc_interrupt_disable();
    }
    /* Start notification and idle timer */
    else
    {
        enable_sds = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
       if(WICED_BT_SUCCESS != wiced_start_timer(&notification_timer_handle, NOTFICATION_TIME_MS))
       {
           WICED_BT_TRACE("Notification timer start failed\n\r");
       }
       if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, IDLE_TIME_S))
       {
           WICED_BT_TRACE("Idle timer start failed\n\r");
       }

    }
}

/**
 * Function         notification_timer_callback
 *
 * @brief           Notification timer callback
 * @param[in] user_data                : arg (not used)
 *
 * @return                          : None
 */
void notification_timer_callback(uint32_t arg)
{
    send_sensor_value_notification();
}

/**
 * Function         idle_timer_callback
 *
 * @brief           Idle timer callback
 * @param[in] user_data                : arg (not used)
 *
 * @return                          : None
 */
void idle_timer_callback(uint32_t arg)
{
    UINT32 int_status;
    wiced_result_t timer_result;

    /* Check the current status of the motion sensor interrupt signal */
    int_status = wiced_hal_gpio_get_pin_input_status(ex07_low_power_INTERRUPT);
    WICED_BT_TRACE("Interrupt Value: %x\n\r", int_status);

    /* If interrupt signal not high stop timers, enable interrupt and go to SDS */
    if(0 == int_status)
    {
        if(WICED_BT_SUCCESS != wiced_stop_timer(&notification_timer_handle))
        {
            WICED_BT_TRACE("Notification timer stop failed\n\r");
        }
        if(WICED_BT_SUCCESS != wiced_stop_timer(&idle_timer_handle))
        {
            WICED_BT_TRACE("Idle timer stop failed\n\r");
        }
        ENABLE_GPIO_INTERRUPT;
        enable_sds = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
    }
    /* If interrupt signal high restart Idle timer */
    else
    {
        if( wiced_is_timer_in_use(&idle_timer_handle) )
        {
            if(WICED_BT_SUCCESS != wiced_stop_timer(&idle_timer_handle))
            {
                WICED_BT_TRACE("Idle timer stop failed\n\r");
            }
        }
        if(WICED_BT_SUCCESS != wiced_start_timer(&idle_timer_handle, 10))
        {
            WICED_BT_TRACE("Idle timer start failed\n\r");
        }
        else
        {
            WICED_BT_TRACE("idle timer re started \n\r");
        }
    }
}

/**
 * Function         remove_bond_data_and_start_advertisement
 *
 * @brief           Remove bond data for NVRAM and start ADV
 *
 * @return                          : None
 */
void remove_bond_data_and_start_advertisement(void)
{
    wiced_result_t result;
    uint16_t written_byte;

    if(0 == ex07_low_power_state.conn_id)
    {
        memset(ex07_low_power_state.remote_addr, 0, 6);
        wiced_hal_write_nvram( ex07_low_power_VS_ID, sizeof(ex07_low_power_hostinfo), 0, &result );
        WICED_BT_TRACE("Peer bond data removed\n\r");

        /* Set the advertising params and make the device discoverable */
        ex07_low_power_set_advertisement_data();

        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n\r", result );
    }
}

/**
 * Function         button_cb
 *
 * @brief           Button callback
 *
 * @return                          : None
 */
void button_cb (void* user_data, uint8_t value )
{
    remove_bond_data_and_start_advertisement();
}

/**
 * Function         shutdown_cb
 *
 * @brief                 Shuts down on low battery
 *
 * @return                       : None
 */
void shutdown_cb(void)
{

    WICED_BT_TRACE("Shutting Down\n\r");

    if( wiced_is_timer_in_use(&idle_timer_handle) )
    {
        wiced_stop_timer(&idle_timer_handle);
    }

    if( wiced_is_timer_in_use(&notification_timer_handle) )
    {
        wiced_stop_timer(&notification_timer_handle);
    }

    ex07_low_power_acc_interrupt_disable();

    power_down_ex07_low_power();

    if(0 == ex07_low_power_state.conn_id)
    {
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    }
    else
    {
        wiced_bt_gatt_disconnect(ex07_low_power_state.conn_id);
    }

    DISABLE_GPIO_INTERRUPT;

    enable_sds = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
}

/**
 * Function         puart_rx_interrupt_callback
 *
 * @brief           Callback function for handling Serial terminal inputs from the user
 *
 * @return                       : None
 */
void puart_rx_interrupt_callback(void* unused)
{
    /* There can be at most 16 bytes in the HW FIFO. */
    uint8_t  readbyte;
    wiced_result_t         result;

    wiced_hal_puart_read( &readbyte );

    switch (user_input_state)
    {
        case NO_ACTION:

            break;

        case GET_PASSKEY:
            if ((readbyte>='0')&&(readbyte<='9'))
            {
                passkey = passkey * 10 + readbyte - '0';
                passkey_itr++;
                if (passkey_itr == 6)
                {
                    user_input_state = NO_ACTION;
                    ex07_low_power_passkey_reply (passkey);
                    passkey_itr = 0;
                    passkey = 0;
                }
            }

    }

    wiced_hal_puart_reset_puart_interrupt( );

}

/**
 * Function         ex07_low_power_set_console_input
 *
 * @brief           For initializing the console input via PUART RX Interrupt
 *
 * @return                       : None
 */
void ex07_low_power_set_console_input( void )
{
    /* Turn off flow control */
    wiced_hal_puart_flow_off( );  /* call  to turn on flow control */

    /* BEGIN - puart interrupt */
    wiced_hal_puart_register_interrupt(puart_rx_interrupt_callback);

    /* set watermak level to 1 to receive interrupt up on receiving each byte */
    wiced_hal_puart_set_watermark_level(1);

    /* Turn on Tx */
    wiced_hal_puart_enable_tx( );
}


/*
 *  Pass protocol traces up through the UART
 */
static void ex07_low_power_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}
