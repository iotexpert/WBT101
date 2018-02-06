/* Setup the BLE to connect and provide a custom service:
 *   CapSense  (custom profile - UUID )
 *
 * The CapSense data is read from the shield board over I2C every 100ms.
 *
 * Notifications are sent whenever the data changes if the CCCD is set for the service.
 * */

#include "sparcommon.h"
#include "wiced_transport.h"
#include "wiced_hal_platform.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_trace.h"
#include "ble_gatt.h"
#include "wiced_timer.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_nvram.h"

/*****************************    Constants   *****************************/
/* Timer for reading I2C data from the PSoC */
#define TIMEOUT_IN_MS (100)

/* The I2C address is 8 bit so need to left shift the 7-bit address by 1 */
#define I2C_ADDRESS  (0x42 <<1)
/* Offset for the register that holds CapSense button values */
#define BUTTON_REG  (0x06)
/* Number of buttons - we only use 3 instead of 4 because the phone app only supports 3 buttons */
#define NUM_BUTTONS (3)
/* Mask for the button values from the I2C register (lower 3 bits of the button register) */
#define BUTTON_MASK (0x07)
/* The characteristic we send is a 3 byte array: number of buttons, mask (unused), button data */
#define BUTTON_CHAR_SIZE (3)
#define BUTTON_HEADER_POS (0)
#define BUTTON_MASK_POS (1)
#define BUTTON_VALUE_POS (2)

/* NVSRAM locations available for application data - NVSRAM Volatile Section Identifier */
#define NVSRAM_VSID                    ( WICED_NVRAM_VSID_START )
#define LOCAL_KEYS_NVSRAM_VSID         ( NVSRAM_VSID + 1 )
#define PAIRED_KEYS_NVSRAM_VSID        ( LOCAL_KEYS_NVSRAM_VSID + 1 )

extern const wiced_bt_cfg_settings_t bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t bt_cfg_buf_pools[];

/*****************************    Variables   *****************************/
/* HCI interface configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, 115200 },
    { 0, 0},
    NULL,
    NULL,
    NULL
};

/* Bluetooth variables */
uint16_t  connection_id;    // connection ID referenced by the stack
BD_ADDR  remote_addr;      // remote device address

uint8_t device_name[]          = WICED_DEVICE_NAME;
uint8_t appearance_name[2]     = { BIT16_TO_8(WICED_DEVICE_APPEARANCE) };
char    char_mfr_name_value[]  = { "Cypress" };
char    char_model_num_value[] = { "1234" };
uint8_t char_system_id_value[] = { 0x12, 0x34, 0x56, 0xff, 0xee, 0x9a, 0xbc, 0xde};

/* Data variables and notification flags */
/* CapSense characteristic */
uint8_t capSenseVal[BUTTON_CHAR_SIZE] =        {NUM_BUTTONS, 0};
uint8_t capSenseValPrev[BUTTON_CHAR_SIZE] =    {NUM_BUTTONS, 0};

#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    BD_ADDR   bdaddr;                               /* BD address of the bonded host */
    uint16_t  characteristic_client_configuration;  /* Current value of the client configuration descriptor */
} host_info_t;
#pragma pack()

host_info_t hostinfo;

/*
 * This is the GATT database for the application.  It defines services,
 * characteristics and descriptors. Each attribute in the database has
 * a handle (characteristic has two, one for the characteristic itself,
 * another for the value).  The handles are used by the peer to access
 * attributes, and can be used locally by the application - for example,
 * to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also
 * has permissions which identify if and how peer is allowed to read or
 * write into it.  All handles do not need to be sequential, but need to
 * be in ascending order.
 */
const uint8_t gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_GAP_SERVICE, UUID_SERVICE_GAP ),

        // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16( HANDLE_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16( HANDLE_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    // Declare proprietary CapSense Service with 128 bit UUID
    PRIMARY_SERVICE_UUID128( HANDLE_CAPSENSE_SERVICE, UUID_CAPSENSE_SERVICE ),

        // Declare characteristic used to notify a change
        CHARACTERISTIC_UUID128( HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS, HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS_VAL,
            UUID_CAPSENSE_CHARACTERISTIC, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE ),

            // Declare client characteristic configuration descriptor.
            // Value of the descriptor can be modified by the client
            // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_CAPSENSE_SERVICE_CHAR_CFG_DESC, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16( HANDLE_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

        // Characteristic Manufacturer Name
        CHARACTERISTIC_UUID16( HANDLE_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Characteristic Model Number
        CHARACTERISTIC_UUID16( HANDLE_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        // Characteristic System ID
        CHARACTERISTIC_UUID16( HANDLE_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),
};

/* Structure for a GATT characteristic (attributes) */
typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

/* GATT Database Characteristic (Attribute) Array */
attribute_t gauAttributes[] =
{
    { HANDLE_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof(device_name),         device_name },
    { HANDLE_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(appearance_name),       appearance_name },
    { HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS_VAL, sizeof(capSenseVal), capSenseVal},
    { HANDLE_CAPSENSE_SERVICE_CHAR_CFG_DESC, sizeof(hostinfo.characteristic_client_configuration), (void*)&hostinfo.characteristic_client_configuration},
    { HANDLE_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(char_mfr_name_value),   char_mfr_name_value },
    { HANDLE_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(char_model_num_value),  char_model_num_value },
    { HANDLE_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(char_system_id_value),  char_system_id_value },
};

/* Timer structure used to periodically read data from the shield over I2C */
wiced_timer_t ms_timer;

/* I2C variables */
uint8 i2cOffset = BUTTON_REG;

struct {
    uint8_t buttonVal;
} __attribute__((packed)) i2cReadBuf;


/*****************************    Function Prototypes   *******************/
static void                    application_init( void );  /* Initialize the application */
static wiced_result_t          bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t  gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t  gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static wiced_bt_gatt_status_t  gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t  gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                    send_message( void );
static attribute_t *           gatts_get_attribute( uint16_t handle );
static void                    getPsocI2c(uint32_t arg);  /* This is a timer callback function used to get I2C data from PSoC */
static void                    bt_set_advertisement_data( void ); /* Setup BLE advertisement packets */

/*****************************    Function Implementations   *********************/
/***********************************************************************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg ); /* Initialize HCI interface */
    /* Send DEBUG messages over the PUART */
    /* Note - for these pins to be connected to the serial device you must set the following switches: */
    /* SW5.1 OFF, SW5.2 ON, SW5.3 OFF, SW5.4 ON */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
    WICED_BT_TRACE("\r"); // Go to start of line after trace init message
    wiced_bt_stack_init( bt_cback, &bt_cfg_settings, bt_cfg_buf_pools ); /* Start up BLE stack */
}

/***********************************************************************************/
/* This function initializes the application.
 * It is executed in the BTM_ENABLED_EVT management callback. */
void application_init( void )
{
    wiced_bt_gatt_status_t      gatt_status;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t link_keys;
    uint8_t                     *p;

    hostinfo.characteristic_client_configuration = 0; /* Start with notifications off */

    /* Initialize wiced app */
    wiced_bt_app_init();

    /* Configure GPIO P2 to drive high so that the reset to the shield is not pulled down */
    wiced_hal_gpio_init( );
    wiced_hal_gpio_configure_pin( WICED_P02, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    /* Initialize I2C interface to the PSoC */
    /* Configure I2C block and pins*/
    wiced_hal_i2c_init(WICED_I2C_SDA_I2S_DOUT_PCM_OUT_SCL_I2S_DIN_PCM_IN);
    /* Write the offset for the button register */
    wiced_hal_i2c_write(&i2cOffset , 1, I2C_ADDRESS);

    /* Start a timer to read the I2C data every 100ms */
    if ( wiced_init_timer(&ms_timer, &getPsocI2c, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER ) == WICED_SUCCESS )
    {
        wiced_start_timer( &ms_timer, TIMEOUT_IN_MS );
    }

    /* Enable privacy to advertise with RPA (Resolvable Private Address) */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );

    /* Load the address resolution DB with the keys stored in the NVRAM */
    /* If no client has connected previously, then this read will fail */
    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( PAIRED_KEYS_NVSRAM_VSID, sizeof(wiced_bt_device_link_keys_t), p, &result);
    if(result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys, link_keys.key_data.ble_addr_type );
    }
    WICED_BT_TRACE("\tRead paired keys from NVSRAM and add to address resolution %B result:%d \r\n", p, result );

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( gatts_callback );

    WICED_BT_TRACE( "\tGATT Init: wiced_bt_gatt_register: %d\r\n", gatt_status );

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init( gatt_database, sizeof(gatt_database) );

    WICED_BT_TRACE("\tGATT Init: wiced_bt_gatt_db_init %d\r\n", gatt_status);

    /* Set the advertising params and make the device discoverable */
    bt_set_advertisement_data();

    /* Start connectable high duty cycle advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, NULL, NULL );
    WICED_BT_TRACE("\t\tStart Advertisements: %d\r\n", result );
}

/***********************************************************************************/
/* Callback function for Bluetooth Stack events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("BT_Callback: %d:\t", event );

    switch( event )
    {
        /* Bluetooth stack enabled */
        case BTM_ENABLED_EVT:
            WICED_BT_TRACE( "Bluetooth Enabled\n\r");
            application_init(); /* Initialize and start other application functionality */
            break;

        /* IO capabilities of the device request from the client */
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE( "Pairing Requested\n\r");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d\n\r", p_info->reason);

            if ( p_info->reason == WICED_BT_SUCCESS ) /* Bonding successful */
            {
                /* Pack the data to be stored into the hostinfo structure */
                memcpy( hostinfo.bdaddr, remote_addr, sizeof( BD_ADDR ) );

                /* Write to NVRAM */
                wiced_hal_write_nvram( NVSRAM_VSID, sizeof(hostinfo), (uint8_t*)&hostinfo, &result );
                WICED_BT_TRACE("\tbonding info save to NVRAM\n\r");
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
             p_status = &p_event_data->encryption_status;
             WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d\n\r", p_status->bd_addr, p_status->result);

             /* Connection has been encrypted meaning that we have correct/paired device restore values in the database */
             wiced_hal_read_nvram( NVSRAM_VSID, sizeof(hostinfo), (uint8_t*)&hostinfo, &(p_status->result) );

         break;

            /* Security request from the client */
            case BTM_SECURITY_REQUEST_EVT:
                WICED_BT_TRACE( "Security Requested\n\r");
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
                break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
                /* save keys to NVRAM */
                WICED_BT_TRACE( "Paired Device Key Update\n\r");
                p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
                wiced_hal_write_nvram ( PAIRED_KEYS_NVSRAM_VSID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
                WICED_BT_TRACE("\tkeys save to NVRAM %B result: %d \n\r", p_keys, result);
            break;

         case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
                /* read keys from NVRAM */
                 WICED_BT_TRACE( "Paired Device Key Request\n\r");
                p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
                wiced_hal_read_nvram( PAIRED_KEYS_NVSRAM_VSID, sizeof(wiced_bt_device_link_keys_t), p_keys, &result );
                WICED_BT_TRACE("\tkeys read from NVRAM %B result: %d \n\r", p_keys, result);
            break;

        /* Update of local keys - save to NVSRAM */
        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
                WICED_BT_TRACE( "Local Identity Key Update\n\r");
                p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
                wiced_hal_write_nvram ( LOCAL_KEYS_NVSRAM_VSID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
                /* Result is the number of bytes written */
                WICED_BT_TRACE("\tlocal keys save to NVRAM result: %d \n\r", result);
                break;

        /* Request for local keys - read from NVSRAM */
        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE( "Local Identity Key Request\n\r");
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( LOCAL_KEYS_NVSRAM_VSID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            /* Result is the number of bytes read */
            WICED_BT_TRACE("\tlocal keys read from NVRAM result: %d \n\r",  result);
            break;

        /* Advertising state change */
        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n\r", *p_mode);
            /* If advertising is off and we are not connected, turn it back on */
            if ( *p_mode == BTM_BLE_ADVERT_OFF && !connection_id)
            {
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, NULL, NULL );
            }
            break;

        default:
            WICED_BT_TRACE( "Default\n\r");
            break;
    }
    return result;
}

/***********************************************************************************/
/* Callback function for Bluetooth GATT events */
wiced_bt_gatt_status_t gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_SUCCESS;

    /* This structure holds the connection status information */
    wiced_bt_gatt_connection_status_t *p_status;
    p_status = &(p_data->connection_status);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT: /* Process connect/disconnect events */
            /* Check to see if this is a connect or disconnect */
            if ( p_status->connected )
            {
                /* On a connect, stop advertising, and initialize everything */
                WICED_BT_TRACE( "\tGATT Callback: connection_up %B id:%d\r\n", p_status->bd_addr, p_status->conn_id);

                /* Update the connection handler.  Save address of the connected device. */
                connection_id = p_status->conn_id;
                memcpy(remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

                /* GJL: This starts bonding when a connection happens. Otherwise the peer has to initiate it. */
                //GJL wiced_bt_dev_sec_bond( p_status->bd_addr,p_status->addr_type,p_status->transport, 0, NULL );

                /* Stop advertising */
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
                WICED_BT_TRACE( "\t\tStop Advertisements%d\r\n", result );
            }
            else
            {
                /* On a disconnect, reset everything and restart advertising */
                WICED_BT_TRACE( "\tGATT Callback: connection_down conn_id:%d reason:%d\r\n", connection_id, p_status->reason );

                /* Resetting the device info */
                connection_id = 0;
                memset( remote_addr, 0, 6 );
                hostinfo.characteristic_client_configuration = 0;

                /* Restart advertisements */
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, NULL, NULL );
                WICED_BT_TRACE( "\t\tStart Advertisements: %d\r\n", result );
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT: /* Process GATT data requests */
            result = gatts_req_cb( &p_data->attribute_request );
            break;

        default:
            break;
    }
    return result;
}

/***********************************************************************************/
/* Process GATT requests from the peer - we will process read and write requests only */
wiced_bt_gatt_status_t gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    WICED_BT_TRACE( "\tGATT Request: gatts_req_cb. connection %d, type %d\r\n", p_data->conn_id, p_data->request_type );

    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = gatts_req_write_handler( p_data->conn_id, &(p_data->data.write_req) );
            break;
        default:
            break;
    }

    return result;
}


/***********************************************************************************/
/* Process Read request or command from peer device */
wiced_bt_gatt_status_t gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = gatts_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("\t\tGATT Read: read_hndlr attr not found hdl:%x\r\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("\t\tGATT Read: read_hndlr conn_id:%d hdl:%x offset:%d len:%d\r\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/***********************************************************************************/
/* Process write request or write command from peer device */
wiced_bt_gatt_status_t gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    WICED_BT_TRACE("\t\tGATT Write: write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\r\n", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    switch ( p_data->handle )
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification */
    case HANDLE_CAPSENSE_SERVICE_CHAR_CFG_DESC:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );

        /* Save value to NVRAM */
        wiced_result_t temp_result;
        wiced_hal_write_nvram( NVSRAM_VSID, sizeof(hostinfo), (uint8_t*)&hostinfo, &temp_result );
        WICED_BT_TRACE("\t\tWrite CCCD value to NVRAM\n\r");

        break;

    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return result;
}

/***********************************************************************************/
/* Find attribute description by handle */
attribute_t * gatts_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
        }
    }
    WICED_BT_TRACE( "\t\tAttribute not found:%x\r\n", handle );
    return NULL;
}

/***********************************************************************************/
/* Set the Advertisement packet data */
void bt_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_TX_POWER;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = ( uint8_t* ) &bt_cfg_settings.max_pwr_db_val;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (char *)bt_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )bt_cfg_settings.device_name ;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/***********************************************************************************/
/* The function invoked on timeout of the timer.
 * It gets temperature and CapSense button data from the PSoC*/
void getPsocI2c( uint32_t arg )
{
    /* Read I2C data */
    wiced_hal_i2c_read((char *) &i2cReadBuf , sizeof(i2cReadBuf), I2C_ADDRESS);

    /* Copy Button values from I2C register to the CapSense GATT array */
    capSenseVal[BUTTON_VALUE_POS] = (i2cReadBuf.buttonVal & BUTTON_MASK);

    /* Check to see if any button values have changed and if so send the value */
    if(capSenseValPrev[BUTTON_VALUE_POS] != capSenseVal[BUTTON_VALUE_POS])
    {
        capSenseValPrev[BUTTON_VALUE_POS] = capSenseVal[BUTTON_VALUE_POS];
        WICED_BT_TRACE("\t\t\tCapSense Button Value Change: %x\r\n",capSenseVal[BUTTON_VALUE_POS]);
        send_message();
    }
}

/***********************************************************************************/
/* Check if client has registered for notification and send message if appropriate */
void send_message( void )
{
    /* Check if connection is up. If not, do nothing */
    if ( connection_id != 0)
    {
        /* If client has registered for notifications, send the value */
        if ( hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
        {
            wiced_bt_gatt_send_notification(connection_id, HANDLE_CAPSENSE_SERVICE_CHAR_BUTTONS_VAL, sizeof(capSenseVal), capSenseVal );
            WICED_BT_TRACE( "\tSend Notification: sending CapSense value\r\n");
        }
    }
}
