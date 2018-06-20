#pragma once
#include "wiced.h"
#include "wiced_bt_ble.h"

typedef struct
{
    wiced_bt_device_address_t       remote_bd_addr;                         /**< Device address */
    uint8_t                         ble_addr_type;                          /**< LE Address type */
    wiced_bt_dev_ble_evt_type_t     ble_evt_type;                           /**< Scan result event type */
    int8_t                          rssi;                                   /**< Set to #BTM_INQ_RES_IGNORE_RSSI, if not valid */
    uint8_t                         flag;
    uint8_t                         data[31];
} scan_device_t;

scan_device_t *dt_findDevice(wiced_bt_device_address_t *bdaddr);
scan_device_t *dt_addDevice(wiced_bt_ble_scan_results_t *scanDev, uint8_t *advData);
uint32_t dt_advGetLength(uint8_t *p_adv_data);
void dt_printDeviceOneLine(scan_device_t *device);
void dt_printDeviceTableOneLine();
void dt_printDeviceTableMultiLine();
