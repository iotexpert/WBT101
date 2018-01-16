/*
 * $ Copyright Cypress Semiconductor $
 */

/*
 * @file afe_shield_gatt_db.c
 */

#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "afe_shield_gatt_db.h"

/*************************************************************************************
** GATT server definitions
*************************************************************************************/

const uint8_t gatt_database[] = // Define GATT database
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

    /* Primary Service 'Environmental Sensing' */
    PRIMARY_SERVICE_UUID16 (HDLS_ENVIRONMENTAL_SENSING, UUID_SERVICE_ENVIRONMENTAL_SENSING),

        /* Characteristic 'Temperature' */
        CHARACTERISTIC_UUID16 (HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE, HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,
            UUID_CHARACTERISTIC_TEMPERATURE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
            LEGATTDB_PERM_READABLE),

            /* Client Characteristic Configuration Descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ ),

            /* Environment Sensing Trigger Settings Descriptor */
            CHAR_DESCRIPTOR_UUID16 (HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING,
                UUID_DESCRIPTOR_ENVIRONMENT_SENSING_TRIGGER_SETTING, LEGATTDB_PERM_READABLE),

        /* Characteristic 'Humidity' */
        CHARACTERISTIC_UUID16 (HDLC_ENVIRONMENTAL_SENSING_HUMIDITY, HDLC_ENVIRONMENTAL_SENSING_HUMIDITY_VALUE,
            UUID_CHARACTERISTIC_HUMIDITY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY,
            LEGATTDB_PERM_READABLE),

            /* Client Characteristic Configuration Descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_ENVIRONMENTAL_SENSING_HUMIDITY_CLIENT_CONFIGURATION,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ ),

            /* Environment Sensing Trigger Settings Descriptor */
            CHAR_DESCRIPTOR_UUID16 (HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING_2,
                UUID_DESCRIPTOR_ENVIRONMENT_SENSING_TRIGGER_SETTING, LEGATTDB_PERM_READABLE),
};
// Length of the GATT database
const uint16_t gatt_database_len = sizeof(gatt_database);

uint8_t afe_shield_device_name[]          = { 't', 'h', 'e', 'r', 'm', 'i','s', 't', 'o', 'r' };
uint8_t afe_shield_appearance[]           = { BIT16_TO_8(APPEARANCE_GENERIC_THERMOMETER) };
uint8_t afe_shield_last_temp_reading[]         = { 0x00, 0x00 };
uint8_t afe_shield_last_hum_reading[]         = { 0x00, 0x00 };
uint8_t afe_shield_client_configuration[] = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};

// Use a fixed time interval between transmissions once per 1 second
#define ENVIROMENTAL_SENSING_TRIGGER_SETTING_USE_FIXED_TIME_INTERVAL    1
uint8_t afe_shield_trigger_setting[] = { ENVIROMENTAL_SENSING_TRIGGER_SETTING_USE_FIXED_TIME_INTERVAL, 0x01, 0x00, 0x00 };

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
wiced_bt_gatt_data_t afe_shield_gatt_db_ext_attr_tbl[] =
        {
            /* { attribute handle,                                      length, offset, attribute data } */
            { HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,                        11, 11, afe_shield_device_name },
            { HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,                         2, 2, afe_shield_appearance },
            { HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE,                 2, 2, afe_shield_last_temp_reading },
            { HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION,  2, 2, afe_shield_client_configuration },
            { HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING,                   4, 4, afe_shield_trigger_setting },
            { HDLC_ENVIRONMENTAL_SENSING_HUMIDITY_VALUE,                    2, 2, afe_shield_last_hum_reading },
            { HDLD_ENVIRONMENTAL_SENSING_HUMIDITY_CLIENT_CONFIGURATION,     2, 2, afe_shield_client_configuration },
            { HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING_2,                 4, 4, afe_shield_trigger_setting },
        };

const uint16_t afe_shield_gatt_db_ext_attr_tbl_size = (sizeof(afe_shield_gatt_db_ext_attr_tbl) / sizeof (wiced_bt_gatt_data_t));
