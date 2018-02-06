#ifndef _BLE_GATT_H_
#define _BLE_GATT_H_

#define WICED_DEVICE_NAME           ("GJL_E02_BLE_CONN")
#define WICED_DEVICE_APPEARANCE     (APPEARANCE_GENERIC_TAG)

/* UUID values for the Custom CapSense Service - these bytes are in reverse order (i.e. little endian) */
#define UUID_CAPSENSE_SERVICE         0x31, 0x01, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xB5, 0xCA, 0x03, 0x00
#define UUID_CAPSENSE_CHARACTERISTIC  0x31, 0x01, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xA3, 0xCA, 0x03, 0x00

/* Definitions for the GATT database handles - these must be sequential */
typedef enum
{
    HANDLE_GATT_SERVICE = 0x1, // service handle

    HANDLE_GAP_SERVICE = 0x10, // service handle
        HANDLE_GAP_SERVICE_CHAR_DEV_NAME, // characteristic handle
        HANDLE_GAP_SERVICE_CHAR_DEV_NAME_VAL, // char value handle

        HANDLE_GAP_SERVICE_CHAR_DEV_APPEARANCE, // characteristic handle
        HANDLE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,// char value handle

    HANDLE_CAPSENSE_SERVICE = 0x30,
        HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS, // characteristic handle
        HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS_VAL, // char value handle
        HANDLE_CAPSENSE_SERVICE_CHAR_CFG_DESC, // char config desc handle

    HANDLE_DEV_INFO_SERVICE = 0x50,
        HANDLE_DEV_INFO_SERVICE_CHAR_MFR_NAME, // characteristic handle
        HANDLE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,// char value handle

        HANDLE_DEV_INFO_SERVICE_CHAR_MODEL_NUM, // characteristic handle
        HANDLE_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,// char value handle

        HANDLE_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, // characteristic handle
        HANDLE_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,// char value handle
}gatt_db_tags;

#endif // _BLE_GATT_H_
