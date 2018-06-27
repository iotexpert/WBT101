/* ----- Imports ----- */
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"

#include "AdvScanner.h"
#include "company_ids.h"
#include "decode_functions.h"
#include "device_table.h"
#include "ring_buffer.h"


/* ----- Definitions ----- */
#define PAGE_SIZE (7)
#define HEADER    "  #  |      Address      | Len |    Name    | E | I | C | Seen | RSSI "
#define PAD       "     |                   |     |            |   |   |   |      |      | "
#define FIRST_BAR "-----+-------------------+-----+------------+---+---+---+------+------+"
#define EXTEND    "-----------------"
#define SHORT_BAR "---------------------------------------------------------------------------------------------"


/* ----- Function Prototypes ----- */
void blePrintAdvPacketData(uint8_t *data,int len,char *pad);


/* ----- Variable declarations ----- */
uint8_t page_num_m = 0;
uint8_t page_num_r = 0;


/* Clear the terminal and reset the cursor position */
void clear_terminal()
{
    // Esc[2J clears the screen
    uint8_t seq1[4] = {0x1B,'[','2','J'};
    wiced_hal_puart_print((char*)seq1);

    // Esc[H moves the cursor to the top left corner
    uint8_t seq2[3] = {0x1B,'[','H'};
    wiced_hal_puart_print((char*)seq2);
}

/* Print unprocessed device data */
void printDeviceOneLine(scan_device_t *device, uint32_t index, wiced_bool_t extra_data)
{
    /* Print the index, address, and data length */
    uint32_t length;
    length = dt_advGetLength(device->data);
    WICED_BT_TRACE("%4d | %B| %02d  | ", index, device->remote_bd_addr, (int)length);

    /* Print the name */
    uint8_t name_len;
    uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,&name_len);
    if(!name_start)
        name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_SHORT,&name_len);

    if(name_start)
    {
        for(int i=0;i<((name_len>10)?10:name_len);i++)
            WICED_BT_TRACE("%c",name_start[i]);
        for(int i=name_len ; i<10;i++)
            WICED_BT_TRACE(" ");
        WICED_BT_TRACE(" | ");
    }
    else
        WICED_BT_TRACE("           | ");

    /* Check for Eddystone, iBeacon, and Cypress devices */
    // Eddystone prefix: 02 01 06 03 03 AA FE
    if(
            device->data[0] == 0x02 &&
            device->data[1] == 0x01 &&
            device->data[2] == 0x06 &&
            device->data[3] == 0x03 &&
            device->data[4] == 0x03 &&
            device->data[5] == 0xAA &&
            device->data[6] == 0xFE
    )
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    // iBeacon prefix: 02 01 06 1A FF 4C 00 02
    if(
            device->data[0] == 0x02 &&
            device->data[1] == 0x01 &&
            device->data[4] == 0xFF &&
            device->data[5] == 0x4C &&
            device->data[6] == 0x00 &&
            device->data[7] == 0x02)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    // Cypress Company prefix: 02 01 06 1A FF 01 31
    if(
            device->data[4] == 0xFF &&
            device->data[5] == 0x01 &&
            device->data[6] == 0x31)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    if(extra_data == WICED_FALSE)
        WICED_BT_TRACE("  ");

    // Print the time since the device was last seen
    uint32_t time_dif = current_time() - device->time_stamp;
    if(time_dif < 60){
        WICED_BT_TRACE(" %2ds", time_dif);
    }else if(time_dif < 3600){
        time_dif/=60;
        WICED_BT_TRACE(" %2dm", time_dif);
    }else if(time_dif < 86400){
        time_dif/=3600;
        WICED_BT_TRACE(" %2dh", time_dif);
    }else if(time_dif < 864000){
        time_dif/=86400;
        WICED_BT_TRACE("  %dd", time_dif);
    }else{
        WICED_BT_TRACE(" >9d");
    }

    if(extra_data == WICED_FALSE)
    {
        WICED_BT_TRACE("\n\r");
        return;
    }

    WICED_BT_TRACE(" | ");

    // Print the RSSI of the device
    WICED_BT_TRACE("%4d | ", device->rssi);

    // Print the device data
    for(int i = 0; i < length && i < 31; i++)
        WICED_BT_TRACE("%02X ", device->data[i]);
    WICED_BT_TRACE("\n\r");

    if(length>31)
        WICED_BT_TRACE("Length Error\n\r");
}

/* Print a table of unprocessed device data */
void printDeviceTableOneLine()
{
    scan_device_t *devices = dt_getTable();

    // Print table header
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE("%s| Data\n\r", HEADER);
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);

    //Print device data
    for(int i=0;i<dt_getNumDevices();i++)
        printDeviceOneLine(&devices[i], i, WICED_TRUE);
}

/* Increment/Decrement multiline table page */
void incrementPageNum_m()
{
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE;
    if(dt_getNumDevices() % PAGE_SIZE)
        max_pages++;
    if(page_num_m < max_pages - 1)
        page_num_m++;
}
void decrementPageNum_m()
{
    if(page_num_m > 0)
        page_num_m--;
}

/* Increment/Decrement recent packet table page */
void incrementPageNum_r()
{
    uint8_t max_pages = 1;
    if(rb_size() > 6)
        max_pages++;
    if(rb_size() > 12)
        max_pages++;
    if(page_num_r < max_pages - 1)
        page_num_r++;
}
void decrementPageNum_r()
{
    if(page_num_r > 0)
        page_num_r--;
}

/* Reset table page numbers */
void reset_tables()
{
    page_num_r = 0;
    page_num_m = 0;
}

/* Print a table of both processed and unprocessed device data */
void printDeviceTableMultiLine()
{
    // Clear the screen
    clear_terminal();

    // Retrieve a pointer to the table
    scan_device_t *devices = dt_getTable();
    uint32_t len, index;

    // Print the table header
    WICED_BT_TRACE("%s%s%s\n\r", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sData\n\r", PAD);
    WICED_BT_TRACE("%s+%s%s\n\r", HEADER, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sDecoded Data\n\r", PAD);

    // Print device data
    char *pad = "     |                   |     |            |   |   |   |      |      +";
    for(int i = 0; i < PAGE_SIZE && (index = i+PAGE_SIZE*page_num_m) < dt_getNumDevices(); i++)
    {
        WICED_BT_TRACE("%s%s%s\n\r", FIRST_BAR, SHORT_BAR, EXTEND);
        len = dt_advGetLength(devices[index].data);
        printDeviceOneLine(&devices[index], index, WICED_TRUE);
        WICED_BT_TRACE("%s%s%s\n\r", pad, SHORT_BAR, EXTEND);
        blePrintAdvPacketData(devices[index].data,len,PAD);
    }

    //Print page numbers
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE;
    if(dt_getNumDevices() % PAGE_SIZE)
        max_pages++;
    WICED_BT_TRACE("Page %d out of %d\n\r", page_num_m + 1, max_pages);
}

/* Print the sixteen most recent advertising packets of the filtered device */
void printRecentFilterData()
{
    // Clear the screen
    clear_terminal();

    //Retrieve a pointer to the filtered device
    scan_device_t *devices = dt_getTable();
    uint32_t focusDevice = dt_getFocus();

    // Table headers
    WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("----- Recent Filter Data -----\n\r");
    WICED_BT_TRACE( "\n\r" );
    //WICED_BT_TRACE("----------- Device -----------\n\r");
    //WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n\r");
    WICED_BT_TRACE("  #  |      Address      | Len |    Name    | E | I | C | Last Seen\n\r");
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n\r");

    // Print unprocessed device data
    printDeviceOneLine(&devices[focusDevice], focusDevice, WICED_FALSE);
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n\r");
    //WICED_BT_TRACE( "\n\r" );
    //WICED_BT_TRACE("------------ Data ------------\n\r");

    // Print the 18 most recent advertising packets (pages of 6)
    rb_print_num(page_num_r*6, 6);

    WICED_BT_TRACE( "\n\r" );

    //Print page numbers
    uint8_t max_pages = 1;
    if(rb_size() > 6)
        max_pages++;
    if(rb_size() > 12)
        max_pages++;
    WICED_BT_TRACE("Page %d out of %d\n\r", page_num_r + 1, max_pages);
}

void printMostRecentFilterData()
{
    WICED_BT_TRACE("\n\r");

    // Print the table header
    WICED_BT_TRACE("%s%s%s\n\r", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sData\n\r", PAD);
    WICED_BT_TRACE("%s+%s%s\n\r", HEADER, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sDecoded Data\n\r", PAD);

    uint32_t len;
    scan_device_t *devices = dt_getTable();
    uint32_t focusDevice = dt_getFocus();
    char *pad = "     |                   |     |            |   |   |   |      |      +";

    // Print device data
    WICED_BT_TRACE("%s%s%s\n\r", FIRST_BAR, SHORT_BAR, EXTEND);
    len = dt_advGetLength(devices[focusDevice].data);
    printDeviceOneLine(&devices[focusDevice], focusDevice, WICED_TRUE);
    WICED_BT_TRACE("%s%s%s\n\r", pad, SHORT_BAR, EXTEND);
    blePrintAdvPacketData(devices[focusDevice].data,len,PAD);

    WICED_BT_TRACE("%s%s%s\n\r", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("\n\r");
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
        WICED_BT_TRACE("\n\r");
    }
}
