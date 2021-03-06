/***************************************************************************//**
* File Name: cycfg_gatt_db.h
* Version: 1.1
*
* Description:
* Definitions for constants used in the device's GATT database and function
* prototypes.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 1.1.0 build 291
*
********************************************************************************
* Copyright 2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS               0x1800u
#define __UUID_CHARACTERISTIC_DEVICE_NAME           0x2A00u
#define __UUID_CHARACTERISTIC_APPEARANCE            0x2A01u
#define __UUID_SERVICE_GENERIC_ATTRIBUTE            0x1801u
#define __UUID_SERVICE_MODUS                        0x58u, 0xAFu, 0xCBu, 0x1Bu, 0xAAu, 0x03u, 0x77u, 0xBFu, 0x30u, 0x4Eu, 0xA1u, 0x23u, 0x48u, 0x54u, 0x80u, 0x85u
#define __UUID_CHARACTERISTIC_MODUS_COUNTER         0x8Cu, 0xE9u, 0xD8u, 0x21u, 0x5Fu, 0x00u, 0xB9u, 0x9Du, 0xBEu, 0x4Au, 0xA4u, 0xCCu, 0x1Au, 0x98u, 0x2Bu, 0x10u
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902u
#define __UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION    0x2901u

/* Service Generic Access */
#define HDLS_GAP                                    0x0001u
/* Characteristic Device Name */
#define HDLC_GAP_DEVICE_NAME                        0x0002u
#define HDLC_GAP_DEVICE_NAME_VALUE                  0x0003u
/* Characteristic Appearance */
#define HDLC_GAP_APPEARANCE                         0x0004u
#define HDLC_GAP_APPEARANCE_VALUE                   0x0005u

/* Service Generic Attribute */
#define HDLS_GATT                                   0x0006u

/* Service Modus */
#define HDLS_MODUS                                  0x0007u
/* Characteristic Counter */
#define HDLC_MODUS_COUNTER                          0x0008u
#define HDLC_MODUS_COUNTER_VALUE                    0x0009u
/* Descriptor Client Characteristic Configuration */
#define HDLD_MODUS_COUNTER_CLIENT_CHAR_CONFIG       0x000Au
/* Descriptor Characteristic User Description */
#define HDLD_MODUS_COUNTER_CHAR_USER_DESCRIPTION    0x000Bu

/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

/* External definitions */
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern const uint16_t app_gap_device_name_len;
extern uint8_t app_gap_appearance[];
extern const uint16_t app_gap_appearance_len;
extern uint8_t app_modus_counter[];
extern const uint16_t app_modus_counter_len;
extern uint8_t app_modus_counter_client_char_config[];
extern const uint16_t app_modus_counter_client_char_config_len;
extern uint8_t app_modus_counter_char_user_description[];
extern const uint16_t app_modus_counter_char_user_description_len;

#endif /* CYCFG_GATT_DB_H */

