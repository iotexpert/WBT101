#include "wiced_bt_trace.h"

#include "company_ids.h"

void bleDecodeAdInfo(uint8_t *bytes);
void bleFormat128bitUUID(uint8_t *bytes);
void bleFormat32bitUUID(uint8_t *bytes);
void bleFormat16bitUUID(uint8_t *bytes);

/* Data decoding functions and table */
struct advDecode_t {
    uint8_t code;
    char *name;
    void (*fcn)(uint8_t *bytes);
};

void bleAdInfoDecodeName(uint8_t *bytes)
{
    for(int i = 0; i < bytes[0]; i++){
        WICED_BT_TRACE("%c",bytes[1+i]);
    }
}
void bleAdInfoDecodeUnknown(char *buff, uint8_t *bytes)
{
    WICED_BT_TRACE("Len:%02X Type:%02X",bytes[0],bytes[1]);
}

void bleAdInfoDecodeFlags(uint8_t *bytes)
{
    int count = 0;

    WICED_BT_TRACE("%02X ",bytes[2]);

    if(bytes[2] & 0x01)
        WICED_BT_TRACE("LE Ltd Discoverable ");

    if(bytes[2] & 0x02)
        WICED_BT_TRACE("LE General Discoverable ");

    if(bytes[2] & 0x04)
        WICED_BT_TRACE("BR/EDR Not Supported ");

    if(bytes[2] & 0x08)
        WICED_BT_TRACE("BR/EDR Controller ");

    if(bytes[2] & 0x10)
        WICED_BT_TRACE("BR/EDR Host ");
}

void bleAdInfoDumpBytesOffset(uint8_t *bytes,uint32_t offset)
{
    int i;
    for(i=offset;i<bytes[0]-1;i++)
        WICED_BT_TRACE("%02X ",bytes[2+i]);
}

void bleAdInfoDumpBytes(uint8_t *bytes)
{
    int i;
    for(i=0;i<bytes[0]-1;i++)
        WICED_BT_TRACE("%02X ",bytes[2+i]);
}

void bleAdInfoDecodeMfgData(uint8_t *bytes)
{
    uint16_t mfg;
    mfg = bytes[2] | bytes[3]<<8;
    WICED_BT_TRACE("%s ",getCompanyName(mfg));
    if(mfg == 0x004C && bytes[4] == 0x02)
    {
        WICED_BT_TRACE("iBeacon ");
        bleAdInfoDumpBytesOffset(bytes,3);
    }
    else
        bleAdInfoDumpBytesOffset(bytes,2);
}

//0xXX:-127 to +127dBm
//Note: when the TX Power Level tag is not present,
//the TX power level of the packet is unknown.
void bleAdInfoDecodePowerData(uint8_t *bytes)
{
    uint16_t power;
    power = bytes[0];
    WICED_BT_TRACE("%d ",(signed int)power);
}

void bleAdInfoDecode16bitServiceUUID(uint8_t *bytes)
{
    bleFormat16bitUUID(&bytes[2]);
}

void bleAdInfoDecode32bitServiceUUID(uint8_t *bytes)
{
    bleFormat128bitUUID(&bytes[2]);
}

void bleAdInfoDecode128bitServiceUUID( uint8_t *bytes)
{
    bleFormat128bitUUID(&bytes[2]);
}

void bleAdInfoDecodePublicAddress(uint8_t *bytes)
{
    bleAdInfoDumpBytes(bytes);
}

// https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
struct advDecode_t advDecodeArray[] = {
        {0x01, "Flags",bleAdInfoDecodeFlags},
        {0x02, "16-bit Service UUID",bleAdInfoDecode16bitServiceUUID},
        {0x03, "16-bit Service UUID",bleAdInfoDecode16bitServiceUUID},
        {0x04, "32-bit Service UUID",bleAdInfoDecode32bitServiceUUID},
        {0x05, "32-bit Service UUID",bleAdInfoDecode32bitServiceUUID},
        {0x06, "128-bit Service UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x07, "128-bit Service UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x08, "Short Name", bleAdInfoDecodeName},
        {0x09, "Complete Name", bleAdInfoDecodeName},
        {0x0A, "Tx Power Level", bleAdInfoDecodePowerData},
        {0x0D, "Device Class", bleAdInfoDumpBytes},
        {0x0E, "Pairing Hash C", bleAdInfoDumpBytes},
        {0x0E, "Pairing Hash C-192", bleAdInfoDumpBytes},
        {0x0F, "Pairing Randomizer R", bleAdInfoDumpBytes},
        {0x0F, "Pairing Randomizer R-192", bleAdInfoDumpBytes},
        {0x10, "Device ID", bleAdInfoDumpBytes},
        {0x10, "Security Manager TK Value", bleAdInfoDumpBytes},
        {0x11, "Security Manager Out of Band Flags", bleAdInfoDumpBytes},
        {0x12, "Slave Connection Interval Range", bleAdInfoDumpBytes},
        {0x14, "16-bit Service Solicitation UUIDs", bleAdInfoDecode16bitServiceUUID},
        {0x15, "128-bit Service Solicitation UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x16, "Service Data", bleAdInfoDumpBytes},
        {0x16, "16-bit Service Data UUID", bleAdInfoDumpBytes},
        {0x17, "Public Target Address", bleAdInfoDumpBytes},
        {0x18, "Random Target Address", bleAdInfoDumpBytes},
        {0x19, "Appearance", bleAdInfoDumpBytes},
        {0x1A, "Advertising Interval", bleAdInfoDumpBytes},
        {0x1B, "LE Bluetooth Device Address", bleAdInfoDumpBytes},
        {0x1C, "LE Role", bleAdInfoDumpBytes},
        {0x1D, "Simple Pairing Hash C-256", bleAdInfoDumpBytes},
        {0x1E, "Simple Pairing Randomizer R-256", bleAdInfoDumpBytes},
        {0x1F, "32-bit Service Solitication UUIDs",bleAdInfoDecode32bitServiceUUID},
        {0x20, "32-bit Service Data UUID",bleAdInfoDecode32bitServiceUUID},
        {0x21, "128-bit Service Data UUID",bleAdInfoDecode128bitServiceUUID},
        {0x22, "LE Secure Connections Confirmation Value",bleAdInfoDumpBytes},
        {0x23, "LE Secure Connections Random Value",bleAdInfoDumpBytes},
        {0x24, "URI",bleAdInfoDumpBytes},
        {0x25, "Indoor Positioning",bleAdInfoDumpBytes},
        {0x26, "Transport Discovery Data",bleAdInfoDumpBytes},
        {0x27, "LE Supported Features",bleAdInfoDumpBytes},
        {0x28, "Channel Map Update Indication",bleAdInfoDumpBytes},
        {0x29, "PB-ADV",bleAdInfoDumpBytes},
        {0x3D, "3D Information Data",bleAdInfoDumpBytes},
        {0xFF, "MFG Data", bleAdInfoDecodeMfgData}
};

// Iterate over the whole ADV Pack and print out each field as a row
void bleDecodeAdInfo(uint8_t *bytes)
{
    int numElements = sizeof(advDecodeArray)/sizeof(struct advDecode_t);
    int i;

    for(i = 0; i < numElements; i++)
    {
        if(bytes[1] == advDecodeArray[i].code)
        {
            WICED_BT_TRACE("%s ",advDecodeArray[i].name);
            (*advDecodeArray[i].fcn)(bytes);
            return;
        }
    }
    WICED_BT_TRACE("Unknown");
    return;
}
void bleFormat128bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE(
            "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
            bytes[15],
            bytes[14],
            bytes[13],
            bytes[12],
            bytes[11],
            bytes[10],
            bytes[9],
            bytes[8],
            bytes[7],
            bytes[6],
            bytes[5],
            bytes[4],
            bytes[3],
            bytes[2],
            bytes[1],
            bytes[0]
    );
}
void bleFormat32bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE(
            "%02X%02X%02X%02X",
            bytes[3],
            bytes[2],
            bytes[1],
            bytes[0]
    );
}
void bleFormat16bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE(
            "%02X%02X",
            bytes[1],
            bytes[0]
    );
}
