/* $ Copyright Cypress Semiconductor $
 */

/*
 * @file ex07_low_power_gatt_db.h
 */

#ifndef __GATT_DATABASE_H__
#define __GATT_DATABASE_H__

#include <stdint.h>
#include "wiced_bt_gatt.h"
#include "ex07_low_power_hw.h"

/******************************************************************************
 *                                Typedefs
 ******************************************************************************/
/* Structure to store data */
typedef struct
{
    uint8_t header;
    union sensor_data_t
    {
        Type3Axisint16 sensor_values[3];
        int16_t humidity_temperature[4];
    } sensor_data;
} __attribute__((packed)) data_packet;

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    uint8_t   dev_prebonded;      /* stay connected or disconnect after all messages are sent */
    BD_ADDR   bdaddr;                                /* BD address of the bonded host */
    uint16_t  sensor_value_characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    uint16_t  battery_characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    uint8_t   address_type;
    wiced_bt_device_link_keys_t link_keys; /* Value of the link keys */
    wiced_bt_local_identity_keys_t local_keys; /* value of local keys */
} host_info_t;
#pragma pack()

typedef struct
{
    BD_ADDR   remote_addr;              /* remote peer device address */
    uint16_t  conn_id;                  /* connection ID referenced by the stack */
} ex07_low_power_state_t;

/******************************************************************************
 *                             External Variables
 ******************************************************************************/
extern const uint8_t gatt_database[];
extern uint16_t gatt_database_len;
extern ex07_low_power_state_t ex07_low_power_state;
extern host_info_t ex07_low_power_hostinfo;
extern uint8_t ex07_low_power_device_name[];
extern uint8_t ex07_low_power_appearance_name[2];
extern data_packet ex07_low_power_char_notify_value;
extern uint8_t ex07_low_power_client_configuration[];
extern uint8_t ex07_low_power_char_mfr_name_value[];
extern uint8_t ex07_low_power_char_model_num_value[];
extern uint8_t ex07_low_power_char_system_id_value[];
extern uint8_t battery_level;
extern wiced_bt_gatt_data_t ex07_low_power_gatt_db_ext_attr_tbl[];
extern const uint16_t ex07_low_power_gatt_db_ext_attr_tbl_size;
extern UINT8 ex07_low_power_char_notify_value_old[];

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/* ***** Primary Service 'Generic Attribute' */
#define HDLS_GENERIC_ATTRIBUTE                                      0x0001

/* ***** Primary Service 'Generic Access' */
#define HDLS_GENERIC_ACCESS                                         0x0014
/* ----- Characteristic 'Device Name' */
#define HDLC_GENERIC_ACCESS_DEVICE_NAME                             0x0015
#define HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE                       0x0016
/* ----- Characteristic 'Appearance' */
#define HDLC_GENERIC_ACCESS_APPEARANCE                              0x0017
#define HDLC_GENERIC_ACCESS_APPEARANCE_VALUE                        0x0018
/* ----- Primary Service 'MOTION SENSOR' */
#define HDLS_ex07_low_power                                          0x0028
/* ----- Characteristic 'Notify' */
#define HDLC_ex07_low_power_NOTIFY                                   0x0029
#define HDLC_ex07_low_power_NOTIFY_VALUE                             0x002A
/* ----- CCCD */
#define HDLD_ex07_low_power_CLIENT_CONFIGURATION                     0x002B
/* ----- Primary Service 'Device Info' */
#define HDLS_DEVICE_INFO                                            0x004D
/* ----- Characteristic 'Manufacturer Name' */
#define HDLC_MANUFACTURER_NAME                                     0x004E
#define HDLC_MANUFACTURER_NAME_VALUE                               0x004F
/* ----- Characteristic 'Manufacturer Name' */
#define HDLC_MODEL_NUMBER                                          0x0050
#define HDLC_MODEL_NUMBER_VALUE                                    0x0051
/* ----- Characteristic 'System ID' */
#define HDLC_SYSTEM_ID                                             0x0052
#define HDLC_SYSTEM_ID_VALUE                                       0x0053
/* ----- Primary Service 'MOTION SENSOR' */
#define HDLS_BATTERY_SERVICE                                       0x0061
 /* ----- Characteristic 'System ID' */
#define HDLC_BATTERY                                              0x0062
#define HDLC_BATTERY_VALUE                                        0x0063
/* ----- CCCD */
#define HDLD_ex07_low_power_BATTERY_CLIENT_CONFIGURATION           0x0064

/* UUIDs for custom service */
#define UUID_SERVICE_ex07_low_power                  0x85, 0x89, 0x06, 0x8b, 0xc1, 0xbd, 0xdc, 0xa5, 0x84, 0x49, 0xb6, 0x87, 0xb6, 0x98, 0x92, 0x73
#define UUID_CHARACTERISTIC_ex07_low_power_NOTIFY    0x59, 0xa4, 0xad, 0xea, 0xa1, 0xfe, 0x53, 0xb5, 0x3e, 0x41, 0x55, 0x3b, 0x13, 0x91, 0xef, 0x33

#endif /* __GATT_DATABASE_H__ */
