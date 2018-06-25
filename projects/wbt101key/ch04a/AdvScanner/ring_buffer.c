#include <stdio.h>
#include "wiced.h"
#include "wiced_bt_trace.h"

#include "print_functions.h"
#include "device_table.h"

#define RING_BUFFER_SIZE (17)

static uint8_t ring_buffer[31][RING_BUFFER_SIZE];
static uint8_t ring_start = 0;
static uint8_t ring_end = 0;
static uint8_t ring_size = 0;

void rb_insert(uint8_t *data)
{
    memcpy(ring_buffer[ring_end], data, 31);
    ring_end = (ring_end+1)%RING_BUFFER_SIZE;
    if(ring_end == ring_start)
        ring_start = (ring_start+1)%RING_BUFFER_SIZE;
    if(ring_size < RING_BUFFER_SIZE)
        ring_size++;
}

void rb_reset()
{
    ring_start = 0;
    ring_end = 0;
    ring_size = 0;
}

void rb_print()
{
    uint32_t len;

    if(ring_size == 0)
        WICED_BT_TRACE("No recent data\n\r");
    for(int i = 0; i < ring_size && i < RING_BUFFER_SIZE-1; i++)
    {
        if(i != 0)
            WICED_BT_TRACE("\n\r");
        uint8_t *data = ring_buffer[(i+ring_start)%RING_BUFFER_SIZE];
        int length = dt_advGetLength(data);
        for(int j=0; j < length && j < 31; j++)
        {
            WICED_BT_TRACE("%02X ",data[j]);
        }
        WICED_BT_TRACE("\n\r");
        len = dt_advGetLength(data);
        blePrintAdvPacketData(data,len,"");
    }
}
