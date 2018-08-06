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
 * This file has the GATT database
 *
 */

#include "wiced_bt_fw_upgrade.h"
#include "spar_utils.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "ex07_low_power_gatt_db.h"
#include "wiced_bt_firmware_upgrade.h"

/*************************************************************************************
** GATT server definitions
*************************************************************************************/

const uint8_t gatt_database[] = /* Define GATT database */
{
    /* Primary Service 'Generic Attribute' */
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ATTRIBUTE, UUID_SERVICE_GATT),

    /* Primary Service 'Generic Access' */
    PRIMARY_SERVICE_UUID16 (HDLS_GENERIC_ACCESS, UUID_SERVICE_GAP),

        /* Characteristic 'Device Name' */
        CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_DEVICE_NAME, HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

        /* Characteristic 'Appearance' */
        CHARACTERISTIC_UUID16 (HDLC_GENERIC_ACCESS_APPEARANCE, HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Primary Service 'Motion Sensor' */
    /*Handle 0x28: WICED Sense Service.
      This is the main proprietary service of WICED SEnse. Note that
      UUID of the vendor specific service is 16 bytes, unlike standard Bluetooth
      UUIDs which are 2 bytes.  _UUID128 version of the macro should be used. */
    PRIMARY_SERVICE_UUID128 (HDLS_ex07_low_power, UUID_SERVICE_ex07_low_power),

        /* Characteristic 'Sensor notification' */
        /*Handle 0x29: characteristic Sensor Notification, handle 0x2a characteristic value
          we support both notification and indication.  Peer need to allow notifications
          or indications by writing in the Characteristic Client Configuration Descriptor
          (see handle 2b below).  Note that UUID of the vendor specific characteristic is
          16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
          of the macro should be used */
        CHARACTERISTIC_UUID128 (HDLC_ex07_low_power_NOTIFY, HDLC_ex07_low_power_NOTIFY_VALUE, UUID_CHARACTERISTIC_ex07_low_power_NOTIFY,
                                LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
                                LEGATTDB_PERM_READABLE ),

            /* Client Characteristic Configuration Descriptor */
            /* Handle 0x2b: Characteristic Client Configuration Descriptor.
               This is standard GATT characteristic descriptor.  2 byte value 0 means that
               message to the client is disabled.  Peer can write value 1 or 2 to enable
               notifications or indications respectively.  Note _WRITABLE in the macro.  This
               means that attribute can be written by the peer. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ex07_low_power_CLIENT_CONFIGURATION,
                                             UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                             LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

    /* Primary Service 'Device Info' */
    /* Handle 0x4d: Device Info service
       Device Information service helps peer to identify manufacture or vendor
       of the device.  It is required for some types of the devices (for example HID,
       and medical, and optional for others.  There are a bunch of characteristics
       available, out of which Hello Sensor implements 3. */
    PRIMARY_SERVICE_UUID16 (HDLS_DEVICE_INFO, UUID_SERVICE_DEVICE_INFORMATION),

        /* Characteristic 'Manufacturer Name' */
        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16 (HDLC_MANUFACTURER_NAME, HDLC_MANUFACTURER_NAME_VALUE, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Characteristic 'Model Number' */
        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16 (HDLC_MODEL_NUMBER, HDLC_MODEL_NUMBER_VALUE, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Characteristic 'System ID' */
        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16 (HDLC_SYSTEM_ID, HDLC_SYSTEM_ID_VALUE, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Primary Service 'Battery' */
    /* Handle 0x61: Battery service
    This is an optional service which allows peer to read current battery level. */
    PRIMARY_SERVICE_UUID16 (HDLS_BATTERY_SERVICE, UUID_SERVICE_BATTERY),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16 (HDLC_BATTERY, HDLC_BATTERY_VALUE, UUID_CHARACTERISTIC_BATTERY_LEVEL,
                LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE ),

            /* Client Characteristic Configuration Descriptor */
            /* Handle 0x63: Characteristic Client Configuration Descriptor.
               This is standard GATT characteristic descriptor.  2 byte value 0 means that
               message to the client is disabled.  Peer can write value 1 or 2 to enable
               notifications or indications respectively.  Note _WRITABLE in the macro.  This
               means that attribute can be written by the peer. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ex07_low_power_BATTERY_CLIENT_CONFIGURATION,
                                             UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                             LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

    /* Handle 0xff00: Cypress vendor specific WICED Secure OTA Upgrade Service. */
    PRIMARY_SERVICE_UUID128 (HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),

        /* Handles 0xff01: characteristic Secure Firmware Upgrade Control Point, handle 0xff02 characteristic value. */
        CHARACTERISTIC_UUID128_WRITABLE (HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
                                         UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
                                         LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),

            /* Handle 0xff03: Declare client characteristic configuration descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
                                             UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                             LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),

        /* Handle 0xff04: characteristic OTA firmware upgrade data, handle 0xff05 characteristic
           value. This characteristic is used to send portions of the FW image */
        CHARACTERISTIC_UUID128_WRITABLE (HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
                                         UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
                                         LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),
};

/* Place the motion sensor state in AON RAM to use it after SDS wakeup */
PLACE_DATA_IN_RETENTION_RAM ex07_low_power_state_t ex07_low_power_state;

/* Holds the host info saved in the NVRAM */
host_info_t ex07_low_power_hostinfo;

/* Length of the GATT database */
uint16_t gatt_database_len = sizeof(gatt_database);

/* Place last read sensor data and battery level in AON */
uint8_t ex07_low_power_device_name[]          = {'M','o','t','i','o','n',' ','S','e','n','s','o','r',0x00,};
uint8_t ex07_low_power_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
PLACE_DATA_IN_RETENTION_RAM data_packet ex07_low_power_char_notify_value;
uint8_t ex07_low_power_client_configuration[] = {BIT16_TO_8(GATT_CLIENT_CONFIG_NOTIFICATION | GATT_CLIENT_CONFIG_INDICATION)};
uint8_t ex07_low_power_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
uint8_t ex07_low_power_char_model_num_value[] = { 0xa5,0xdc,0xbd,0xc1,0x8b,0x6,0x89,0x85};
uint8_t ex07_low_power_char_system_id_value[] = { 0xa5,0xdc,0xbd,0xc1,0x8b,0x6,0x89,0x85};
PLACE_DATA_IN_RETENTION_RAM uint8_t battery_level;

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
wiced_bt_gatt_data_t ex07_low_power_gatt_db_ext_attr_tbl[] =
{
    {HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,            sizeof(ex07_low_power_device_name),          sizeof(ex07_low_power_device_name),           ex07_low_power_device_name},
    {HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,             sizeof(ex07_low_power_appearance_name),      sizeof(ex07_low_power_appearance_name),       ex07_low_power_appearance_name},
    {HDLC_ex07_low_power_NOTIFY_VALUE,                  sizeof(ex07_low_power_char_notify_value),    sizeof(ex07_low_power_char_notify_value),     (uint8_t *) &ex07_low_power_char_notify_value},
    {HDLD_ex07_low_power_CLIENT_CONFIGURATION,          2,                                          2,                                           (uint8_t *)&ex07_low_power_hostinfo.sensor_value_characteristic_client_configuration },
    {HDLC_MANUFACTURER_NAME_VALUE,                     sizeof(ex07_low_power_char_mfr_name_value),  sizeof(ex07_low_power_char_mfr_name_value),   ex07_low_power_char_mfr_name_value},
    {HDLC_MODEL_NUMBER_VALUE,                          sizeof(ex07_low_power_char_model_num_value), sizeof(ex07_low_power_char_model_num_value),  ex07_low_power_char_model_num_value},
    {HDLC_SYSTEM_ID_VALUE,                             sizeof(ex07_low_power_char_system_id_value), sizeof(ex07_low_power_char_system_id_value),  ex07_low_power_char_system_id_value},
    {HDLC_BATTERY_VALUE,                               1,                                          1,                                           &battery_level},
    {HDLD_ex07_low_power_BATTERY_CLIENT_CONFIGURATION,  2,                                          2,                                           (uint8_t *)&ex07_low_power_hostinfo.battery_characteristic_client_configuration},
};

const uint16_t ex07_low_power_gatt_db_ext_attr_tbl_size = (sizeof(ex07_low_power_gatt_db_ext_attr_tbl) / sizeof (wiced_bt_gatt_data_t));
