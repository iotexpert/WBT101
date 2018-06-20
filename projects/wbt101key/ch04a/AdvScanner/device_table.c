#include "wiced.h"
#include "device_table.h"
#include "wiced_bt_trace.h"
#include "company_ids.h"
#include <stdio.h>

void bleDecodeAdInfo(uint8_t *bytes);
void bleFormat128bitUUID(uint8_t *bytes);
void bleFormat32bitUUID(uint8_t *bytes);
void bleFormat16bitUUID(uint8_t *bytes);
void blePrintAdvPacketData(uint8_t *data,int len,char *pad);

static scan_device_t devices[64];
static uint32_t numDevices;

scan_device_t *dt_findDevice(wiced_bt_device_address_t *bdaddr)
{
    for(uint32_t i=0;i<numDevices;i++)
    {
        if(memcmp(&devices[i].remote_bd_addr,bdaddr,sizeof(wiced_bt_device_address_t)) == 0)
            return &devices[i];
    }
    return 0;
}

scan_device_t *dt_addDevice(wiced_bt_ble_scan_results_t *scanDev, uint8_t *advData)
{
    scan_device_t *myDev;
    myDev = dt_findDevice(&scanDev->remote_bd_addr);

    if(!myDev)
    {
        myDev = &devices[numDevices++];
        memcpy(&myDev->remote_bd_addr,&scanDev->remote_bd_addr,sizeof(wiced_bt_device_address_t));
        myDev->flag = scanDev->flag;
        myDev->ble_addr_type = scanDev->ble_addr_type;
        myDev->rssi = scanDev->rssi;
        WICED_BT_TRACE("Adding new device %B Scan Type %d\n",scanDev->remote_bd_addr,scanDev->ble_evt_type);
    }

    memcpy(myDev->data,advData,31);

    return myDev;
}


uint32_t dt_advGetLength(uint8_t *p_adv_data)
{

    uint32_t length = 0;

    for(int i=0; p_adv_data[i] && length<31; i += 1+p_adv_data[i])
    {
        length += p_adv_data[i] + 1;
    }
    return length;
  }

void dt_printDeviceOneLine(scan_device_t *device)
{
      uint32_t nameLength;
      uint32_t length;
      length = dt_advGetLength(device->data);

      WICED_BT_TRACE("Addr = %B %02d ",device->remote_bd_addr, (int)length);

      uint8_t name_len;
      uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,&name_len);
      if(!name_start)
      {
          uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_SHORT,&name_len);
      }


      if(name_start)
      {
          for(int i=0;i<((name_len>10)?10:name_len);i++)
          {
              WICED_BT_TRACE("%c",name_start[i]);
          }
          for(int i=name_len ; i<10;i++)
          {
              WICED_BT_TRACE(" ");
          }
          WICED_BT_TRACE("  ");
      }
      else
      {
          WICED_BT_TRACE("            ");
      }

      for(int i=0;i<length;i++)
      {
          WICED_BT_TRACE("%02X ",device->data[i]);

      }
      WICED_BT_TRACE("\n");

      if(length>31)
      {
          WICED_BT_TRACE("Length Error\n");
      }
}

void dt_printDeviceTableOneLine()
{
    for(int i=0;i<numDevices;i++)
    {
        dt_printDeviceOneLine(&devices[i]);
    }

}

#define PAD "                                         "
void dt_printDeviceTableMultiLine()
{
    uint32_t len;

    for(int i=0;i<numDevices;i++)
    {
        len = dt_advGetLength(devices[i].data);
        dt_printDeviceOneLine(&devices[i]);
        blePrintAdvPacketData(devices[i].data,len,PAD);
    }

}

// print each field on one line
// len f# name data

void blePrintAdvPacketData(uint8_t *data,int len,char *pad)
{
    int index=0;

    uint8_t *adInfoPacket;

    while(index<len)
    {
        adInfoPacket = &data[index];
        index += data[index] + 1;

        WICED_BT_TRACE(pad);
        bleDecodeAdInfo(adInfoPacket);
        WICED_BT_TRACE("\n");
    }

}


struct advDecode_t {
    uint8_t code;
    char *name;
    void (*fcn)(uint8_t *bytes);
};

void bleAdInfoDecodeName(uint8_t *bytes)
{

    WICED_BT_TRACE("%.*s",bytes[0],&bytes[1]);
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
    {
        WICED_BT_TRACE("%02X ",bytes[2+i]);
    }

}

void bleAdInfoDumpBytes(uint8_t *bytes)
{
    int i;
    for(i=0;i<bytes[0]-1;i++)
    {
        WICED_BT_TRACE("%02X ",bytes[2+i]);
    }

}

void bleAdInfoDecodeMfgData( uint8_t *bytes)
{
    uint16_t mfg;
    mfg = bytes[2] | bytes[3]<<8;
    WICED_BT_TRACE("%s ",getCompanyName(mfg));
    bleAdInfoDumpBytesOffset(bytes,2);
}

void bleAdInfoDecode16bitServiceUUID( uint8_t *bytes)
{
    bleFormat16bitUUID(&bytes[2]);
}

void bleAdInfoDecode32bitServiceUUID( uint8_t *bytes)
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
    {0x0A, "Tx Power Level", bleAdInfoDumpBytes},
    {0x0D, "Device Class", bleAdInfoDumpBytes},
    {0x0D, "Pairing Hash C", bleAdInfoDumpBytes},
    {0x0E, "Pairing Hash C-192", bleAdInfoDumpBytes},
    {0x0F, "Pairing Randomizer R", bleAdInfoDumpBytes},
    {0x10, "Device ID", bleAdInfoDumpBytes},
    {0x11, "Security Manger Out of Band Flags", bleAdInfoDumpBytes},
    {0x12, "Slave Connection Interval Range", bleAdInfoDumpBytes},
    {0x14, "16-bit Service Solicitation UUIDs", bleAdInfoDecode16bitServiceUUID},
    {0x15, "128-bit Service Solicitation UUIDs", bleAdInfoDecode128bitServiceUUID},
    {0x16, "Service Data", bleAdInfoDumpBytes},
    {0x17, "Public Target Address", bleAdInfoDumpBytes},
    {0x18, "Random Target Address", bleAdInfoDumpBytes},
    {0x18, "Appearance", bleAdInfoDumpBytes},
    {0x1A, "Advertising Interval", bleAdInfoDumpBytes},
    {0x1B, "LE Bluetooth Device Address", bleAdInfoDumpBytes},
    {0x1C, "LE Role", bleAdInfoDumpBytes},
    {0x1D, "Simple Pairing Hash C-256", bleAdInfoDumpBytes},
    {0x1E, "Simple Pairing Randomizer R-256", bleAdInfoDumpBytes},
    {0x1F, "32-bit Service Solitication UUIDs",bleAdInfoDecode32bitServiceUUID},
    {0x20, "32-bit Service Data UUID",bleAdInfoDecode32bitServiceUUID},
    {0x21, "128-bit Service Data UUID",bleAdInfoDecode128bitServiceUUID},
    {0x22, "LE Secure Connection Confirmation Value",bleAdInfoDumpBytes},
    {0x23, "LE Connection Random Value",bleAdInfoDumpBytes},
    {0x24, "URI",bleAdInfoDumpBytes},
    {0x25, "Indoor Positioning",bleAdInfoDumpBytes},
    {0x26, "Transport Discovery Data",bleAdInfoDumpBytes},
    {0x27, "LE Supported Features",bleAdInfoDumpBytes},
    {0x28, "Channel Map Update Indication",bleAdInfoDumpBytes},
    {0x3D, "3D Information Data",bleAdInfoDumpBytes},
    {0xFF, "MFG Data", bleAdInfoDecodeMfgData}
};


// Iterate over the whole ADV Pack and print out each field as a row
void bleDecodeAdInfo(uint8_t *bytes)
{
    int numElements = sizeof(advDecodeArray)/sizeof(struct advDecode_t);
    int i;

    for(i = 0; i<numElements; i++)
    {
        if(bytes[1] == advDecodeArray[i].code)
        {
            WICED_BT_TRACE("%s ",advDecodeArray[i].name );
            //int count=sprintf(buff,"%s:",advDecodeArray[i].name);  // Use the # of bytes to setup string cat
            //buff[count] = ' ';                                    // turn the 0 into a space
            (*advDecodeArray[i].fcn)(bytes);         // ARH this is nuts... just a cheap concatenation
            return;
        }
    }
    WICED_BT_TRACE("Unknown");
    return;


}


void bleFormat128bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
        bytes[15],bytes[14],bytes[13],bytes[12],
        bytes[11],bytes[10],
        bytes[9],bytes[8],
        bytes[7],bytes[6],
        bytes[5],bytes[4],bytes[3],bytes[2],bytes[1],bytes[0]);

}

void bleFormat32bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X%02X%02X",
                bytes[3],bytes[2],bytes[1],bytes[0]);

}

void bleFormat16bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X",
        bytes[1],bytes[0]);

}

