/*
 * $ Copyright Cypress Semiconductor $
 */

/*
 * @file afe_shield_gatt_db.h
 */

#ifndef __GATT_DATABASE_H__
#define __GATT_DATABASE_H__

#include <stdint.h>
#include "wiced_bt_gatt.h"

// ***** Primary Service 'Generic Attribute'
#define HDLS_GENERIC_ATTRIBUTE                                      0x0001

// ***** Primary Service 'Generic Access'
#define HDLS_GENERIC_ACCESS                                         0x0014
// ----- Characteristic 'Device Name'
#define HDLC_GENERIC_ACCESS_DEVICE_NAME                             0x0015
#define HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE                       0x0016
// ----- Characteristic 'Appearance'
#define HDLC_GENERIC_ACCESS_APPEARANCE                              0x0017
#define HDLC_GENERIC_ACCESS_APPEARANCE_VALUE                        0x0018

// ***** Primary Service 'Environmental Sensing'
#define HDLS_ENVIRONMENTAL_SENSING                                  0x0028
// ----- Characteristic 'Temperature'
#define HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE                      0x0029
#define HDLC_ENVIRONMENTAL_SENSING_TEMPERATURE_VALUE                0x002A
// Client Configuration Descriptors
#define HDLD_ENVIRONMENTAL_SENSING_TEMPERATURE_CLIENT_CONFIGURATION 0x002B
#define HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING                  0x002C
// ----- Characteristic 'Humidity'
#define HDLC_ENVIRONMENTAL_SENSING_HUMIDITY                      0x002D
#define HDLC_ENVIRONMENTAL_SENSING_HUMIDITY_VALUE                0x002E
// Client Configuration Descriptors
#define HDLD_ENVIRONMENTAL_SENSING_HUMIDITY_CLIENT_CONFIGURATION 0x002F
#define HDLD_ENVIRONMENTAL_SENSING_TRIGGER_SETTING_2                  0x0030

// External definitions
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern wiced_bt_gatt_data_t afe_shield_gatt_db_ext_attr_tbl[];
extern const uint16_t afe_shield_gatt_db_ext_attr_tbl_size;
extern uint8_t afe_shield_device_name[];
extern uint8_t afe_shield_appearance[];
extern uint8_t afe_shield_last_temp_reading[2];
extern uint8_t afe_shield_last_hum_reading[2];
extern uint8_t afe_shield_client_configuration[];
extern uint8_t BT_LOCAL_NAME[];
extern const uint16_t BT_LOCAL_NAME_CAPACITY;

#endif /* __GATT_DATABASE_H__ */
