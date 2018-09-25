/*******************************************************************
 * Imports
 ******************************************************************/
#include "sparcommon.h"
#include "hci_control_api.h"

#include "wiced.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_rtos.h"
#include "wiced_transport.h"
#include "wiced_memory.h"


/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2
#define TYPE_TIMEOUT            2

/*******************************************************************
 * Variable Definitions
 ******************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
// Transport pool for sending RFCOMM data to host
static wiced_transport_buffer_pool_t* transport_pool = NULL;
static wiced_bt_ble_scan_type_t scan_state = BTM_BLE_SCAN_TYPE_NONE;

static uint16_t conn_id;
static uint16_t capsenseHandle;
static uint16_t capsenseCCCDHandle=0x12;
static uint16_t ledHandle;

static const uint8_t ledUUID[] = { 0xF1u, 0x34u, 0x9Bu, 0x5Fu, 0x80u, 0x00u, 0x00u, 0x80u, 0x00u, 0x10u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
static const uint8_t capsenseUUID[] = { 0xF2u, 0x34u, 0x9Bu, 0x5Fu, 0x80u, 0x00u, 0x00u, 0x80u, 0x00u, 0x10u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                  change_scan_state      (wiced_bt_ble_scan_type_t new_state);
static wiced_bt_dev_status_t advscanner_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                  rx_cback                          ( void *data );

/*******************************************************************
 * Macro Definitions
 ******************************************************************/
// Macro to extract uint16_t from little-endian byte array
#define LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(byte_array) \
        (uint16_t)( ((byte_array)[0] | ((byte_array)[1] << 8)) )



/*******************************************************************
 * Function Definitions
 ******************************************************************/


/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
void application_start(void)
{
    /* Initialize the transport configuration */
//    wiced_transport_init( &transport_cfg );

    /* Initialize Transport Buffer Pool */
    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, TRANS_UART_BUFFER_COUNT );

    /* Set Debug UART as WICED_ROUTE_DEBUG_TO_PUART to see debug traces on Peripheral UART (PUART) */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    /* Initialize Bluetooth Controller and Host Stack */
    wiced_bt_stack_init(advscanner_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

void newAdv(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{

    uint8_t length;
    uint8_t *serviceUUID = wiced_bt_ble_check_advertising_data(p_adv_data,BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE,&length);
    if(serviceUUID)
    {
        uint8_t capsenseService[] = {0xF0 ,0x34 ,0x9B ,0x5F ,0x80 ,0x00 ,0x00 ,0x80 ,0x00 ,0x10 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 };
        if(memcmp(capsenseService,serviceUUID,16)== 0)
        {
            WICED_BT_TRACE("Found ADV Packet with Service UUID attempting connection %B\r\n",p_scan_result->remote_bd_addr);
            WICED_BT_TRACE("Address Type = %d\r\n",p_scan_result->ble_addr_type);
            WICED_BT_TRACE("Event Type = %d\r\n",p_scan_result->ble_evt_type);
            WICED_BT_TRACE("Flag = %d\r\n",p_scan_result->flag);
            wiced_result_t result = wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,p_scan_result->ble_addr_type,BLE_CONN_MODE_HIGH_DUTY,WICED_TRUE);
            WICED_BT_TRACE("Result = %s\r\n",(result==WICED_TRUE)?"TRUE":"FALSE");
            change_scan_state(BTM_BLE_SCAN_TYPE_NONE);
        }
    }

}

/* This function is executed in the BTM_ENABLED_EVT management callback */
static void change_scan_state(wiced_bt_ble_scan_type_t new_state)
{
    if(new_state != scan_state)
    {
        wiced_bt_ble_scan(new_state,FALSE,newAdv);
        scan_state = new_state;
    }
}


wiced_bt_gatt_status_t gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_SUCCESS;
    uint16_t val;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        if ( p_data->connection_status.connected )
        {
            WICED_BT_TRACE("Connected\r\n");

            uint8_t dev_role;
            wiced_bt_gatt_connection_status_t *p_conn_status = &p_data->connection_status;

            conn_id         =  p_conn_status->conn_id;
            WICED_BT_TRACE("Connection ID=%d\r\n",conn_id);
        }
        else
        {
            WICED_BT_TRACE(("Disconnected\r\n"));
            conn_id = 0;
        }
        break;


     //   wiced_bt_gatt_event_data_t data;

#if 0
        typedef struct
        {
            uint16_t                                conn_id;            /**< ID of the connection */
            wiced_bt_gatt_optype_t                  op;                 /**< Type of operation completed (see @link wiced_bt_gatt_optype_e wiced_bt_gatt_optype_t @endlink) */
            wiced_bt_gatt_status_t                  status;             /**< Status of operation */
            wiced_bt_gatt_operation_complete_rsp_t  response_data;      /**< Response data (dependent on optype) */
        } wiced_bt_gatt_operation_complete_t;
#endif

    case GATT_OPERATION_CPLT_EVT:

        val = (p_data->operation_complete.response_data.att_value.p_data[0]);
        //+ p_data->operation_complete.response_data.att_value.p_data[1];

        WICED_BT_TRACE("Gatt Event Complete Conn=%d Op=%d status=0x%X Handle=0x%X len=%d value=%X\r\n",
                p_data->operation_complete.conn_id,
                p_data->operation_complete.op,
                p_data->operation_complete.status,
                p_data->operation_complete.response_data.handle,
                p_data->operation_complete.response_data.att_value.len,
                val);


        break;

    case GATT_DISCOVERY_RESULT_EVT:

        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("Discovered Services All Event\r\n");
            WICED_BT_TRACE("Start=%X End=%X\r\n",GATT_DISCOVERY_RESULT_SERVICE_START_HANDLE(p_data),GATT_DISCOVERY_RESULT_SERVICE_END_HANDLE(p_data));
            WICED_BT_TRACE("UUID Len=%d ",GATT_DISCOVERY_RESULT_SERVICE_UUID_LEN(p_data));


            for(int i=0;i<GATT_DISCOVERY_RESULT_SERVICE_UUID_LEN(p_data);i++)
            {
                WICED_BT_TRACE("%02X ",GATT_DISCOVERY_RESULT_SERVICE_UUID128(p_data)[i]);

            }
            WICED_BT_TRACE("\r\n");
        }


        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS)
        {

            WICED_BT_TRACE("Discovered Characteristic\r\n");
            WICED_BT_TRACE("Handle = %X Len=%d ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data),
                    GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN(p_data));


            for(int i=0;i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN(p_data);i++)
            {
                WICED_BT_TRACE("%02X ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data)[i]);

            }
            WICED_BT_TRACE("\r\n");



#if 0
            if(memcmp(GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data),ledUUID,16) == 0)
            {
                WICED_BT_TRACE("Led Handle=0x%X\r\n",GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data));
                ledHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data);
            }

            if(memcmp(GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data),capsenseUUID,16) == 0)
            {
                WICED_BT_TRACE("CapSense Handle=0x%X\r\n",GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data));
                capsenseHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data);
            }
#endif

        }

        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
        {

            WICED_BT_TRACE("Discovered Characteristic Descriptors\r\n");
            WICED_BT_TRACE("Handle = %X Len=%d ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE(p_data),
                    GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN(p_data));

            for(int i=0;i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN(p_data);i++)
            {
                WICED_BT_TRACE("%02X ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID128(p_data)[i]);

            }
            WICED_BT_TRACE("\r\n");
        }
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        WICED_BT_TRACE("Discovery Event Complete\r\n");
        break;


    case     GATT_ATTRIBUTE_REQUEST_EVT:                         /**< GATT attribute request (from remote client). Event data: #wiced_bt_gatt_attribute_request_t */
        WICED_BT_TRACE("Gatt Attribute Request Event\r\n");

        break;
    case    GATT_CONGESTION_EVT:
    WICED_BT_TRACE("Gatt Attribute Request Event\r\n");
            break;

            /*
    default:
        WICED_BT_TRACE(("Unknown GATT Event %d\r\n",event));
        break; */

    }

    return result;
}

void writeLed(uint8_t val)
{
    if(conn_id == 0 && ledHandle != 0)
        return;


    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + 1 );
    if ( p_write )
        {
            p_write->handle   = ledHandle;
            p_write->offset   = 0;
            p_write->len      = 1;
            p_write->auth_req = GATT_AUTH_REQ_NONE;
            p_write->value[0] = val;

            wiced_bt_gatt_status_t status = wiced_bt_gatt_send_write ( conn_id, GATT_WRITE, p_write );

            WICED_BT_TRACE("wiced_bt_gatt_send_write 0x%X\r\n", status);

            wiced_bt_free_buffer( p_write );
        }

}


/* Bluetooth Management Event Handler */
wiced_bt_dev_status_t advscanner_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */


        WICED_BT_TRACE("Bluetooth Enabled (%s)\n\r",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));



        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n\r", bda);

            if(wiced_bt_gatt_register( gatt_callback ) != WICED_BT_GATT_SUCCESS)
            {
                WICED_BT_TRACE("Unable to register GATT callback\r\n");
            }
            else
            {
                WICED_BT_TRACE("Success register GATT callback\r\n");
            }


            /* Enables scanning */
            //change_scan_state(BTM_BLE_SCAN_TYPE_HIGH_DUTY);
        }
        /* Initialize the UART for input */
        wiced_hal_puart_init( );
        wiced_hal_puart_flow_off( );
        wiced_hal_puart_set_baudrate( 115200 );

        /* Enable receive and the interrupt */
        wiced_hal_puart_register_interrupt( rx_cback );

        /* Set watermark level to 1 to receive interrupt up on receiving each byte */
        wiced_hal_puart_set_watermark_level( 1 );
        wiced_hal_puart_enable_rx();

        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\n\r");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\n\r");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\n\r");
        /* No IO Capabilities on this Platform */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND|BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = 0;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        /* Pairing is Complete */
        p_ble_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE("Pairing Complete %d.\n\r", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n\r", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\n\r");
        /* Device/app-specific TODO: HANDLE PAIRED DEVICE LINK REQUEST KEY - retrieve from NVRAM, etc */
#if 0
        if (advscanner_read_link_keys( &p_event_data->paired_device_link_keys_request ))
        {
            WICED_BT_TRACE("Key Retrieval Success\n\r");
        }
        else
#endif
            /* Until key retrieval implemented above, just fail the request - will cause re-pairing */
        {
            WICED_BT_TRACE("Key Retrieval Failure\n\r");
            status = WICED_BT_ERROR;
        }
        break;
    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /*
         * Request local identity key (get local_identity_keys from NV memory).
         * If successful, return WICED_BT_SUCCESS.
         * Event data: #wiced_bt_local_identity_keys_t
         */
        WICED_BT_TRACE("Local identity key requested\n\r");
        break;
    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        /* Scan State Changed */
        if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_NONE)
            WICED_BT_TRACE("Scanning Disabled\n\r");
        else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_LOW_DUTY)
            WICED_BT_TRACE("Scanning Enabled (Low Duty)\n\r");
        else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_HIGH_DUTY)
            WICED_BT_TRACE("Scanning Enabled (High Duty)\n\r");
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n\r", *p_adv_mode);
        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* Pairing request, TODO: handle confirmation of numeric compare here if desired */
        WICED_BT_TRACE("numeric_value: %d\n\r", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;


    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n\r", event, event);
        break;
    }

    return status;
}
void startServiceDiscovery()
{

    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 1;
    discovery_param.e_handle = 0xFFFF;
    //discovery_param.uuid.uu.uuid128;

    uint8_t capsenseServiceUUID[] = { 0xF0 ,0x34 ,0x9B ,0x5F ,0x80 ,0x00 ,0x00 ,0x80 ,0x00 ,0x10 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 };
    discovery_param.uuid.len = 16;
    memcpy(&discovery_param.uuid.uu.uuid128,capsenseServiceUUID,16);

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id,     GATT_DISCOVER_SERVICES_ALL,   &discovery_param);
    WICED_BT_TRACE("Start Service Discovery 0x%X\r\n",status);

}

void startCharacteristicDiscovery()
{
    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 0x0c;
    discovery_param.e_handle = 0x13;

//    discovery_param.s_handle = 0x01;
//    discovery_param.e_handle = 0x13;

//    uint8_t capsenseCharUUID[] = { 0xF2 ,0x34 ,0x9B ,0x5F ,0x80, 0x00, 0x00, 0x80,0x00 ,0x10 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 };

//    discovery_param.uuid.len = 16;
//    memcpy(&discovery_param.uuid.uu.uuid128,capsenseCharUUID,16);

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id,     GATT_DISCOVER_CHARACTERISTICS,   &discovery_param);
    WICED_BT_TRACE("Start char Discovery 0x%X\r\n",status);
}

void startCharDescriptorDiscovery()
{
    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 0xf;
    discovery_param.e_handle = 0xf;

//    discovery_param.uuid.len = 2;
//    discovery_param.uuid.uu.uuid16 = 0x2902;

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id,     GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,   &discovery_param);
    WICED_BT_TRACE("Start char descriptor discovery 0x%X\r\n",status);

}


void setCCCD()
{
    if(conn_id == 0 || capsenseCCCDHandle == 0)
        return;

    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + 1 );
    if ( p_write )
    {
        p_write->handle   = capsenseCCCDHandle;
        p_write->offset   = 0;
        p_write->len      = 2;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        p_write->value[0] = 1;
        p_write->value[1] = 0;

        wiced_bt_gatt_status_t status = wiced_bt_gatt_send_write ( conn_id, GATT_WRITE, p_write );
        WICED_BT_TRACE("wiced_bt_gatt_send_write 0x%X\r\n", status);
        wiced_bt_free_buffer( p_write );
    }
}

//TODO: Add key to print out most recent packet
/* Interrupt callback function for UART */
void rx_cback( void *data )
{
    uint8_t  readbyte;
    uint32_t focus;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    switch (readbyte)
    {

    case 's':
        wiced_hal_puart_print( "Scan On\n\r" );
        change_scan_state(BTM_BLE_SCAN_TYPE_HIGH_DUTY);
        break;
    case 'S':
        wiced_hal_puart_print( "Scan Off\n\r" );

        change_scan_state(BTM_BLE_SCAN_TYPE_NONE);
        break;

    case '0':
        WICED_BT_TRACE("LED Off\r\n");

        writeLed(0);
    break;

    case '1':
        WICED_BT_TRACE("LED On\r\n");
        writeLed(1);
    break;

    case '2':
            WICED_BT_TRACE("LED On\r\n");
            writeLed(1);
            writeLed(1);
        break;


    case 'd':
        wiced_bt_gatt_disconnect(conn_id);
    break;

    case 'q': // service discovery
        startServiceDiscovery();
        break;

    case 'w':
        startCharacteristicDiscovery();
        break;

    case 'e':
         startCharDescriptorDiscovery();
         break;


    case 'c':
        setCCCD();
        break;

        case '?':
            /* Print help */
            wiced_hal_puart_print( "\n\r" );
            wiced_hal_puart_print( "+------- Available Commands -------+\n\r" );
            wiced_hal_puart_print( "|  c    Set CCCD                   |\n\r" );

            wiced_hal_puart_print( "|  C    Find CCCD                  |\n\r" );
            wiced_hal_puart_print( "|  s    Turn on Scanning           |\n\r" );
            wiced_hal_puart_print( "|  S    Turn off Scanning          |\n\r" );
            wiced_hal_puart_print( "|  0    LED Off                    |\n\r" );
            wiced_hal_puart_print( "|  1    LED On                     |\n\r" );
            wiced_hal_puart_print( "|  d    disconnect                 |\n\r" );
            wiced_hal_puart_print( "|  q    Service Discovery          |\n\r" );
            wiced_hal_puart_print( "|  ?    Print Commands             |\n\r" );
            wiced_hal_puart_print( "+----------------------------------+\n\r" );
            wiced_hal_puart_print( "\n\r" );
            break;
        }
}
