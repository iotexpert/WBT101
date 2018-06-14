/*
 * This file has been automatically generated by the WICED 20719-B1 Designer.
 * SDP database for Single Mode BR/EDR or Dual Mode device.
 *
 */

// TestClassicSpp_sdp_db.c

#include "ex01_spp_sdp_db.h"

#include "wiced_bt_uuid.h"
#include "wiced_bt_sdp.h"

const uint8_t sdp_database[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_1(157),

    // SDP Record for Serial Port
    SDP_ATTR_SEQUENCE_1(84),
        SDP_ATTR_RECORD_HANDLE(HDLR_SERIAL_PORT),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SERIAL_PORT_SCN),
        SDP_ATTR_BROWSE_LIST,
        SDP_ATTR_LANGUAGE_BASE_ATTR_ID_LIST,
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),
        SDP_ATTR_SERVICE_NAME(11),
            'S', 'P', 'P', ' ', 'S', 'e', 'r', 'v', 'i', 'c', 'e',

    // SDP Record for Device ID
    SDP_ATTR_SEQUENCE_1(69),
        SDP_ATTR_RECORD_HANDLE(HDLR_DEVICE_ID),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),
        SDP_ATTR_PROTOCOL_DESC_LIST(1),
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0131),
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0xFFFF),
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0xFFFF),
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),
};

// Length of the SDP database
const uint16_t sdp_database_len = sizeof(sdp_database);
