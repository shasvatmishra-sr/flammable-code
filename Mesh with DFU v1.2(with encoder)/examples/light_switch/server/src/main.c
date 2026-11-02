

#include <stdint.h>
#include <string.h>
#include <math.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"
#include "nrf_log.h"


/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "ble_softdevice_support.h"

#include <boards.h>
#include "nrf_mesh_events.h"
#include "nrf_log.h"
#include "nrf_mesh_dfu.h"
#include "app_util.h"
#include "nrf_mesh_serial.h"
#include "nrf_drv_gpiote.h"

#include "nrf_drv_clock.h"

#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
//#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "nrf_drv_timer.h"
#include "nrfx_ppi.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"

#include "nrf_drv_wdt.h"

#include "nrf_drv_twi.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
 void set_pwm(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
 void speed_pwm(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
 void set_buzz(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

#define RED_LED   15//4
#define GREEN_LED 17//5
#define BLUE_LED  20//NRF_GPIO_PIN_MAP(1,9)

#define SCL1 4//15
#define SDA1 5//17

#define PREHEAT_TIME_MS (1000)
#define FEED_INTERVAL_MS 100


APP_TIMER_DEF(restart_timer);
APP_TIMER_DEF(encoder_timer);
APP_TIMER_DEF(timeout_timer);
APP_TIMER_DEF(device_timer);
APP_TIMER_DEF(send_timer);


#define ZC 4//20
#define BUTTON 11//30
#define TRIAC 1//28

#define INT 31

#define SENSOR_ON NRF_GPIO_PIN_MAP(1,9)
#define FAN NRF_GPIO_PIN_MAP(0,0)


#define RELAY 1 
#define BUZZER 5
bool adc_firstSample = true; 


bool relay_state=false, transition=false, lock=false;
uint8_t brightness=0, temp=0, transition_time=10;
int8_t step=0;
bool manual_override = false;
uint8_t shutdown_time=0;

const float invBeta = 1.00/3455.00;    //thermistor beta
const float adcMax = 1023.00;          //max 10bit value
const float invT0 = 1.00/298.15;       //room temp in kelvin

bool calibrated = false;
bool sensor_alert_flag = false;
static bool send_timer_started = false;
uint16_t adc_x1,adc_x2,adc_temp,count;
uint32_t sum;
uint16_t adc_filterd, threshold = 210,PPM;
float_t Ro;
uint16_t adc_max = 1023;
uint32_t send_time = 120000;
float_t V_in = 3.3;
float_t Rload = 4.7;
float_t Vc = 5.0;
float_t R_in_air;
float_t ratio;



bool debug = false;

const nrfx_timer_t TIMER = NRFX_TIMER_INSTANCE(3);
const nrfx_timer_t TIMER2 = NRFX_TIMER_INSTANCE(4);


#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
//#endif

#define APP_ONOFF_ELEMENT_INDEX     (0)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION      (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                (NRF_MESH_TRANSMIC_SIZE_SMALL)

/***************************GLOBAL VARIABLES****************************************/
uint32_t fw=0, app=0;


uint16_t pwm_seq[4]={0,0,0,0};
uint16_t pwm_buzz[4]={0,0,0,0};
uint16_t pwm_speed[4]={0,0,0,0};


bool relay_status=0, zs_flag=0;
uint32_t delay_on=0, delay_off=0;
mesh_config_entry_id_t entry = {.file = 0x0010, .record = 0x0001};

bool triac_flag = 0;
bool fan_flag = 0;

uint32_t time=0, avg_zc=0;
uint32_t zero_crossing_time[3]={0};
uint8_t counter = 0;
uint8_t tid = 0;
bool zc_set = false;

uint16_t raw_data = 0;



/***********************************************************************************/

/********************PPI Channel***********************************************/
static nrf_ppi_channel_t ppi_channel_timer_capture;
nrf_ppi_channel_t ppi_channel1;
/*********************************************************************/
static nrf_ppi_channel_t ppi_channel_timer_start;
static nrf_ppi_channel_t ppi_channel_timer_stop;
static nrf_ppi_channel_t ppi_channel_pin_high;
static nrf_ppi_channel_t ppi_channel_pin_low;
/**************************Saadc Sensor**************************************************/
static uint8_t current_sensor = 1;
static generic_onoff_server_t m_server;




static void sensor_timer_handler();
static void send_sensor_data(uint8_t sensor_id);
void send_timer_handler(void);
void software_interrupt_init(void);
void temperature_calc(int adcVal, uint8_t* temperature);
void send_adc();
void SensorAlert();

/********************Watchdog handler**********************************/
nrf_drv_wdt_channel_id m_channel_id;

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
//    bsp_board_leds_off();

    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}



/**********************************************************************/

/******************TO save data in persistent memory*******************/
MESH_CONFIG_FILE(m_app_file, 0x0010, MESH_CONFIG_STRATEGY_CONTINUOUS);

#define APP_ENTRY_ID    MESH_CONFIG_ENTRY_ID(0x0010, 0x0001)
/* Live RAM representation of the value */
static uint32_t m_live_value[100] = {0};
static uint32_t app_entry_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint32_t * p_value = (const uint32_t *) p_entry;
    m_live_value[id.record] = *p_value;
    return NRF_SUCCESS;
}
static void app_entry_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint32_t * p_value = (uint32_t *) p_entry;
    *p_value = m_live_value[id.record];
}
MESH_CONFIG_ENTRY(m_app_entry,
                  APP_ENTRY_ID,
                  100, // The entry is singular
                  sizeof(uint32_t),
                  app_entry_setter,
                  app_entry_getter,
                  NULL, // No need for a delete callback
                  false); // There is a default value

/******************************************************************************************/

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_onoff_server_set_cb(const generic_onoff_server_t * p_self, const generic_onoff_set_params_t * p_in);
static void app_light_control_set_cb(const generic_onoff_server_t * p_self, const generic_light_set_params_t * p_in);
static void app_onoff_server_get_cb(const generic_onoff_server_t * p_self, const access_message_rx_meta_t * p_meta, generic_onoff_status_params_t * p_out);
static void app_onoff_server_transition_cb(const generic_onoff_server_t * p_self, const generic_onoff_set_params_t * p_in, const model_transition_t * p_in_transition);
static void app_ack_transaction_status_cb(access_model_handle_t model_handle, void * args, access_reliable_status_t status);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;


static nrf_mesh_evt_handler_t m_evt_handler;

static bool fw_updated_event_is_for_me(const nrf_mesh_evt_dfu_t * p_evt)
{
    switch (p_evt->fw_outdated.transfer.dfu_type)
    {
        case NRF_MESH_DFU_TYPE_APPLICATION:
            return (p_evt->fw_outdated.current.application.app_id == p_evt->fw_outdated.transfer.id.application.app_id &&
                    p_evt->fw_outdated.current.application.company_id == p_evt->fw_outdated.transfer.id.application.company_id &&
                    p_evt->fw_outdated.current.application.app_version < p_evt->fw_outdated.transfer.id.application.app_version);

        case NRF_MESH_DFU_TYPE_BOOTLOADER:
            return (p_evt->fw_outdated.current.bootloader.bl_id == p_evt->fw_outdated.transfer.id.bootloader.bl_id &&
                    p_evt->fw_outdated.current.bootloader.bl_version < p_evt->fw_outdated.transfer.id.bootloader.bl_version);

        case NRF_MESH_DFU_TYPE_SOFTDEVICE:
            return false;

        default:
            return false;
    }
}

static const uint32_t * optimal_bank_address(void)
{
    /* The incoming transfer has to fit on both sides of the bank address: First it needs to fit
     * above the bank address when we receive it, then it needs to fit below the bank address when
     * we install it. We want to put the bank address in the middle of the available application
     * code area, to maximize the potential transfer size we can accept. */
    const uint32_t * p_start;
    uint32_t dummy;
    ERROR_CHECK(mesh_stack_persistence_flash_usage(&p_start, &dummy));

    uint32_t middle_of_app_area = (CODE_START + (intptr_t) p_start) / 2;

    /* The bank can't start in the middle of the application code, and should be page aligned: */
    return (const uint32_t *) ALIGN_VAL(MAX(middle_of_app_area, CODE_END), PAGE_SIZE);
}

static void mesh_evt_handler(const nrf_mesh_evt_t* p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED:
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH:
            if (fw_updated_event_is_for_me(&p_evt->params.dfu))
            {
                const uint32_t * p_bank = optimal_bank_address();
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Requesting DFU transfer with bank at 0x%p\n", p_bank);

                ERROR_CHECK(nrf_mesh_dfu_request(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                                 &p_evt->params.dfu.fw_outdated.transfer.id,
                                                 p_bank));
//                hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            }
            else
            {
                /**
                 * While preparing for the start of the DFU process, the DFU module
                 * will notify about any other ongoing DFU transfers by sending
                 * @ref NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH or
                 * @ref NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED.
                 *
                 * Check the current DFU state to avoid reverting the target state
                 * to the relay state.
                 */

                 __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set to Relay DFU\n");

                nrf_mesh_dfu_transfer_state_t state;
                uint32_t error_code = nrf_mesh_dfu_state_get(&state);
                if (error_code == NRF_SUCCESS && (state.state == NRF_MESH_DFU_STATE_INITIALIZED ||
                                                  state.state == NRF_MESH_DFU_STATE_FIND_FWID ||
                                                  state.state == NRF_MESH_DFU_STATE_RELAY_CANDIDATE ||
                                                  state.state == NRF_MESH_DFU_STATE_RELAY))
                {
                    ERROR_CHECK(nrf_mesh_dfu_relay(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                                    &p_evt->params.dfu.fw_outdated.transfer.id));
                }
            }
            break;

        case NRF_MESH_EVT_DFU_START:
//            hal_led_mask_set(LEDS_MASK_DFU_RUNNING, true);
            break;

        case NRF_MESH_EVT_DFU_END:
//            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
//            hal_led_mask_set(LEDS_MASK_DFU_ENDED, true);
            break;

        case NRF_MESH_EVT_DFU_BANK_AVAILABLE:
            //hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            ERROR_CHECK(nrf_mesh_dfu_bank_flash(p_evt->params.dfu.bank.transfer.dfu_type));
            break;

        default:
            break;

    }
}

static generic_onoff_server_t server;

const generic_onoff_server_callbacks_t server_cbs =
{
   .set_cb = app_onoff_server_set_cb,
   .get_cb = app_onoff_server_get_cb,
   .light_set_cb = app_light_control_set_cb,
   .ack_transaction_status_cb = app_ack_transaction_status_cb
};



static void app_ack_transaction_status_cb(access_model_handle_t model_handle, void * args, access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void node_reset(void)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}


bool relay=0;
uint8_t status_command = 0;
/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const generic_onoff_server_t * p_self, const generic_onoff_set_params_t * p_in)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PWM %d\n", p_in->command);
   if (debug)
   {
    set_pwm(0, 0, 0, 200);
    nrf_delay_ms(200);
    set_pwm(0,0,0,0);
   }
   
         uint8_t data=p_in->on_off;
         uint8_t command = p_in->command;
         uint32_t status = 0;

         if(command==1)
         {
            //status_command = 1;
            app_timer_stop(device_timer);
            app_timer_start(device_timer, APP_TIMER_TICKS(50),(void*)(uint32_t) data);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "In command 1 \n")

         }
         else if(command==2)
         {
            threshold = ((float)data) * 100.0f;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "In command 2 data: %d \n",data);
            entry.record = 0x0003;
            mesh_config_entry_set(entry , &threshold);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"  threshold:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(threshold)); 
         }
         else if(command==3)
         {   
         send_time = data*1000;
         entry.record = 0x0006;
            mesh_config_entry_set(entry , &send_time);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"  send_time:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(send_time)); 
         }

         else if(command==4)
         {  
         }
         else if(command==5)
         {
             //counter_limit = data*10;
         }
         else if(command ==200)
         {
           Ro = (float)data * 1000.0f;
           __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "In command 2 data: %d \n",data);
           entry.record = 0x0005;
           mesh_config_entry_set(entry , &Ro);
           __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"  Ro:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(Ro));
           calibrated = true;

         }
         else if(command==27)
         {
              uint32_t retval=0;
            
              shutdown_time = p_in->on_off;
            
              set_pwm(0,100,0,0);

              retval = proxy_stop();
              //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Disable Proxy %d \n", retval);
              if (retval == NRF_SUCCESS)
              {
                  retval = nrf_mesh_disable();
              }
              //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Disable Mesh %d \n", retval);

              app_timer_stop(restart_timer);
              app_timer_start(restart_timer, APP_TIMER_TICKS(60000), NULL);
         }
         else if(command==250)
         {
            if(debug)
            {
               debug = false;
            }
            else
            {
               debug = true;
            }

             set_pwm(0,0,0,200);
             nrf_delay_ms(300);
             set_pwm(0,0,0,0);
         }
  
}



static void app_light_control_set_cb(const generic_onoff_server_t * p_self, const generic_light_set_params_t * p_in)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Length %d\n", p_in->length);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Command %d\n", p_in->command);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Brightness %d %d %d %d %d %d %d %d\n", p_in->brightness[0], p_in->brightness[1], p_in->brightness[2],
    //                                                                           p_in->brightness[3], p_in->brightness[4], p_in->brightness[5],
    //                                                                           p_in->brightness[6], p_in->brightness[7]);
}


/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const generic_onoff_server_t * p_self, const access_message_rx_meta_t * p_meta, generic_onoff_status_params_t * p_out)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
}

/* Callback for updating the hardware state */
static void app_onoff_server_transition_cb(const generic_onoff_server_t * p_self, const generic_onoff_set_params_t * p_in, const model_transition_t * p_in_transition)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target OnOff: %d\n",
    //                                   transition_time_ms, target_onoff);
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
      server.settings.force_segmented = APP_FORCE_SEGMENTATION;
      server.settings.transmic_size = APP_MIC_SIZE;
      server.settings.p_callbacks = &server_cbs;

      ERROR_CHECK(generic_onoff_server_init(&server, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", server.model_handle);
}

/*************************************************************************************************/

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t-------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) LED state will toggle and inform clients about the state change.\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t-------------------------------------------------------------------------------\n";
#endif


static void button_event_handler(uint32_t button_number)
{

}

static void app_rtt_input_handler(int key)
{
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    for(uint8_t i=0;i<10;i++)
    {
       set_pwm(0,100,0,0);
       nrf_delay_ms(50);
       set_pwm(0,0,0,0);
       nrf_delay_ms(50);
    }
}

static void provisioning_aborted_cb(void)
{
    //hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    //hal_led_blink_stop();
    //hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    //hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
    
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }

    m_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    software_interrupt_init();
//    hal_leds_init();

//#if BUTTON_BOARD
//    ERROR_CHECK(hal_buttons_init(button_event_handler));
//#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    //rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_RELAY_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

}

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       NULL,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void uart_event_handle(app_uart_evt_t * p_event)
{
    
}
void SWI3_EGU3_IRQHandler(void)
{
    send_timer_handler();
}

void software_interrupt_init(void)
{
    NVIC_SetPriority(SWI3_EGU3_IRQn, 6);
    NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);
}

void zc_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
   if(pin==ZC)
   {
         if(counter)
         {
           zero_crossing_time[counter-1] = NRF_TIMER3->CC[1];

           //printf("Zero_crossing %d\n", zero_crossing_time[counter-1]);
           
           if(zero_crossing_time[counter-1]>8000 && zero_crossing_time[counter-1]<12000)
           {
               //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ZC %d %d\n", counter, zero_crossing_time[counter-1]);
               if(counter==3)
               {
                   nrf_drv_gpiote_in_event_disable(ZC);

                   if(avg_zc<zero_crossing_time[2])
                   {
                       time = avg_zc - (abs(avg_zc - zero_crossing_time[2]));
                   }
                   else
                   {
                       time = avg_zc + (abs(avg_zc - zero_crossing_time[2]));
                   }

                   //if(!triac_flag)
                   //{
                       nrfx_ppi_channel_enable(ppi_channel1);

                       nrfx_timer_clear(&TIMER2);
                       nrf_drv_timer_extended_compare(&TIMER2, NRF_TIMER_CC_CHANNEL0, time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true); 
                       nrfx_timer_resume(&TIMER2);

                       triac_flag = true;
                   //}
     
                   zc_set = true;
               }
               else if(counter==2)
               { 
                 avg_zc = (zero_crossing_time[0] + zero_crossing_time[1])/2;
               }

               counter++;
               
               //set_pwm(0,0,0,0);
           }
           else
           {
              counter = 1;
             // set_pwm(0,200,0,0);
           }
         }
         else{
            counter=1;
         }
   }
}


void zc_init()
{
    ret_code_t err_code;

    //err_code = nrf_drv_gpiote_init();
    //APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;                                                  // NRF_GPIO_PIN_NOPULL for version 2 PCB

    err_code = nrf_drv_gpiote_in_init(ZC, &in_config, zc_handler);

    //printf("Error %d\n", err_code);
    APP_ERROR_CHECK(err_code);

    //nrf_drv_gpiote_in_event_enable(ZC, true);
}



uint32_t button_press=0;


bool button1=0, button2=0, button3=0, button4=0;
bool timeout_flag=0;

static void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{  
   uint16_t temp=0; 
   timeout_flag=0;
   uint32_t status;
   //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "P \n");
   //        set_pwm(0,200,0,0);
   if(TIMER_DIFF(button_press, NRF_RTC1->COUNTER) > 6553)
   {
     if(pin==BUTTON)                  
     {  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "P \n");
          set_pwm(0,200,0,0);
         uint32_t timeout = 0;
         while(!nrf_gpio_pin_read(BUTTON) && timeout<1000)
         {
            ++timeout;
            nrf_delay_ms(10);
            nrf_drv_wdt_channel_feed(m_channel_id);
         } 
         
         if(timeout<1000)
         {                
            if(relay)
            {
             set_pwm(0,100,0,0);
            }
            else
            {
             set_pwm(0,0,100,0);
            }
          
          nrf_delay_ms(100);
          set_pwm(0,0,0,0);
         }
         else if(timeout>=1000)
         {
            set_pwm(0,200,0,200);
            if (mesh_stack_is_device_provisioned())
            {
                #if MESH_FEATURE_GATT_PROXY_ENABLED
                                (void) proxy_stop();
                #endif
                mesh_stack_config_clear();
                node_reset();
            }
         }
     } 
    button_press = NRF_RTC1->COUNTER;
   }
}


static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
   
    nrf_drv_gpiote_in_config_t in_config3 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config3.pull = NRF_GPIO_PIN_PULLUP;                                                  // NRF_GPIO_PIN_NOPULL for version 2 PCB

    err_code = nrf_drv_gpiote_in_init(BUTTON, &in_config3, button_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON, true);  
}


static void timer_handler()
{
   nrfx_timer_pause(&TIMER); 
   nrfx_timer_clear(&TIMER);}

static void timer2_handler()
{
       nrfx_timer_pause(&TIMER2); 
       nrfx_timer_clear(&TIMER2);



          if(relay)
           {
                    nrfx_ppi_channel_disable(ppi_channel_pin_high);
                    nrfx_ppi_channel_free(ppi_channel_pin_high);
           }
           else
           {
                   nrfx_ppi_channel_disable(ppi_channel_pin_low);
                   nrfx_ppi_channel_free(ppi_channel_pin_low);
           }
}


void timer_3_init()
{
    ret_code_t err_code = NRF_SUCCESS;

    nrfx_timer_config_t config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    config.frequency = 4;
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err_code = nrf_drv_timer_init(&TIMER, &config, timer_handler);            //Initialise and configure timer
    APP_ERROR_CHECK(err_code);

    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER, 90);                               // calculate timer ticks for a specific time (time_ms) 

    nrf_drv_timer_extended_compare(&TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true); 

    nrfx_timer_enable(&TIMER);
    nrfx_timer_pause(&TIMER);
    nrfx_timer_clear(&TIMER);
}

void timer_4_init()
{
    ret_code_t err_code = NRF_SUCCESS;

    nrfx_timer_config_t config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    config.frequency = 4;
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err_code = nrf_drv_timer_init(&TIMER2, &config, timer2_handler);            //Initialise and configure timer
    APP_ERROR_CHECK(err_code);

    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER, 90);                               // calculate timer ticks for a specific time (time_ms) 

    nrf_drv_timer_extended_compare(&TIMER2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true); 

    nrfx_timer_enable(&TIMER2);
    nrfx_timer_pause(&TIMER2);
    nrfx_timer_clear(&TIMER2);
}



void timeout()
{
 zs_flag = false;
 relay= 0;
 relay_status = true;
}


void device_status(void * p_context)
{
    generic_sensor_status_params_t p_params;
    switch((uint32_t) p_context)
    {
       case 1: //send status
               p_params.sensor_type = 1;
               p_params.attribute_1 = fw;
               p_params.attribute_2 = app;
               p_params.attribute_3 = 0;
               p_params.tid = ++tid;
               access_model_reliable_cancel(server.model_handle);
               generic_device_status_publish(&server, &p_params);
                __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "val\n");
               break;
       case 2: //send node status
            p_params.sensor_type = 18;
            p_params.attribute_1 = adc_x1;
            p_params.attribute_2 = adc_x1>>8;
            p_params.attribute_3 = 0;
            access_model_reliable_cancel(server.model_handle);
            generic_sensor_status_publish(&server, &p_params);
          //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);
          __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "avg2_scaled = %d\n",adc_x1);
          break;

      case 3:  
           { uint8_t rat = (uint8_t)ratio;
            p_params.sensor_type = 18; //124
            p_params.attribute_1 = rat;
            p_params.attribute_2 = PPM;
            p_params.attribute_3 = PPM>>8;
            access_model_reliable_cancel(server.model_handle);
            generic_sensor_status_publish(&server, &p_params);
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published  temp - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);
                 break;
           }
       case 4: //send node status
            p_params.sensor_type = 5;
            p_params.attribute_1 = temp;
            p_params.attribute_2 = threshold;
            p_params.attribute_3 = threshold>>8;
            access_model_reliable_cancel(server.model_handle);
            generic_sensor_status_publish(&server, &p_params);
          //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);
          __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "temp = %d\n",temp);
               break;
       case 26: //Reset mesh
                set_pwm(0,200,0,200);
                if (mesh_stack_is_device_provisioned())
                {
                    #if MESH_FEATURE_GATT_PROXY_ENABLED
                                    (void) proxy_stop();
                    #endif
                    mesh_stack_config_clear();
                    node_reset();
                }
    }
}



void saadc_timer_handler()
{
}

#define SAMPLES_IN_BUFFER 3
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, saadc_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 500);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,timer_compare_event_addr,saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{   if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);

        adc_x1   = p_event->data.done.p_buffer[0];
        adc_x2   = p_event->data.done.p_buffer[1];
        adc_temp = p_event->data.done.p_buffer[2];

        if(adc_firstSample == true)
        {
         adc_filterd = adc_x1;
        }
        else 
        adc_filterd = ((adc_filterd*10) + adc_x1)/10;

       temperature_calc(adc_temp, &temp);

       SensorAlert();
    }
}



/*
  AIN0  p0.02
  AIN1  p0.03
  AIN2  p0.04
  AIN3  p0.05
  AIN4  p0.28
  AIN5  p0.29
  AIN6  p0.30
  AIN7  p0.31
*/



void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config_2 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    nrf_saadc_channel_config_t channel_config_5 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    nrf_saadc_channel_config_t channel_config_6 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

        channel_config_2.gain = NRF_SAADC_GAIN1_4;
        channel_config_5.gain = NRF_SAADC_GAIN1_4;
        channel_config_6.gain = NRF_SAADC_GAIN1_4;
  
  //   channel_config_0.reference = ;
    channel_config_2.reference = NRF_SAADC_REFERENCE_VDD4 ;
    channel_config_5.reference = NRF_SAADC_REFERENCE_VDD4 ;
    channel_config_6.reference = NRF_SAADC_REFERENCE_VDD4;
    
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_config_2);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(5, &channel_config_5);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(6, &channel_config_6);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void start_adc()
{
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();
}

void stop_adc()
{
    nrf_drv_timer_disable(&m_timer);
    nrf_drv_timer_uninit(&m_timer);
    nrf_drv_ppi_channel_disable(m_ppi_channel);
    nrf_drv_ppi_uninit();
    nrf_drv_saadc_abort();
    nrf_drv_saadc_uninit();
    while(nrf_drv_saadc_is_busy());
}

void temperature_calc(int adcVal, uint8_t* temperature)
{
    float voltage;
    float r_ntc;
    float temp_kelvin;

    voltage = ((float)adcVal / adcMax) * 3.3;
    r_ntc = 10000.0 * (voltage / (3.3 - voltage));
    temp_kelvin = 1.0f / (invT0 + invBeta * logf(r_ntc / 10000.0));
    *temperature = temp_kelvin-273.15;
}

void averaging()
{
  sum += adc_filterd; 
  count++;
}
static void send_timer_callback(void * p_context)
{
  (void)p_context;
  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "send_timer callback: alert=%d\n", sensor_alert_flag);
  send_timer_handler();
}
void SensorAlert()
{
  bool alert = adc_filterd > threshold;

  if(!alert)
      averaging();

  if(!send_timer_started || alert != sensor_alert_flag)
  {
      uint32_t interval_ms = alert ? 10000 : send_time;

      if(send_timer_started)
          APP_ERROR_CHECK(app_timer_stop(send_timer));

      if(alert)
      {
          sensor_alert_flag = true;
          NVIC_SetPendingIRQ(SWI3_EGU3_IRQn);
          set_buzz(0, 125, 0, 0);
          set_pwm(0, 250, 0, 0);
      }
      else
      {
          sensor_alert_flag = false;
          set_buzz(0, 0, 0, 0);
          set_pwm(0, 0, 0, 0);
      }

      APP_ERROR_CHECK(app_timer_start(send_timer, APP_TIMER_TICKS(interval_ms), NULL));
      send_timer_started = true;
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "send_timer started: alert=%d interval_ms=%u adc=%u threshold=%u\n",alert, interval_ms, adc_filterd, threshold);
  }
}

void resistance()
{
    float_t adcCal = sensor_alert_flag || count == 0 ? adc_filterd : (float_t)sum/count;

    if(adcCal <= 0.0f || adcCal > adc_max || Ro <= 0.0f)
    {
        R_in_air = 0.0f;
        ratio = 0.0f;
        return;
    }

    float_t Vout = ((adcCal*V_in)/adc_max)*1.532f;
    if(Vout <= 0.0f || Vout >= Vc)
    {
        R_in_air = 0.0f;
        ratio = 0.0f;
        return;
    }

    R_in_air = Rload*(Vc- Vout)/Vout;
    ratio = R_in_air/Ro;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Ro:" NRF_LOG_FLOAT_MARKER,"R_in_air:" NRF_LOG_FLOAT_MARKER,"ratio:" NRF_LOG_FLOAT_MARKER"\n", NRF_LOG_FLOAT(Ro), NRF_LOG_FLOAT(R_in_air), NRF_LOG_FLOAT(ratio));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"Vout:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(Vout));
}

void PPM_cal()
{
  if(ratio <= 0.0f)
      PPM = 0;
  else
  {
      float_t ppm_value = powf(ratio,-0.4771f)*5000.0f;
      PPM = ppm_value >= 65535.0f ? 65535 : (uint16_t)ppm_value;
  }
  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PPM: %d\n",PPM);
}


void send_timer_handler(void){
  if(!calibrated)      
  {
    generic_sensor_status_params_t p_params;
          p_params.sensor_type = 18; //124
          p_params.attribute_1 = adc_x1;
          p_params.attribute_2 = adc_x1>>8;
          p_params.attribute_3 = R_in_air;
          access_model_reliable_cancel(server.model_handle);
          generic_sensor_status_publish(&server, &p_params);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published adc - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);
       __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "adc_x1 = %d\n",adc_x1);
  }
  if(calibrated)
  {
    resistance();
    PPM_cal();
    uint8_t rat = (uint8_t)ratio;
    generic_sensor_status_params_t p_params;
        p_params.sensor_type = 18; //124
        p_params.attribute_1 = rat;
        p_params.attribute_2 = PPM;
        p_params.attribute_3 = PPM>>8;
        access_model_reliable_cancel(server.model_handle);
        generic_sensor_status_publish(&server, &p_params);
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published  cal - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);

  }
  
        generic_sensor_status_params_t p_params;
        p_params.sensor_type = 5; //124
        p_params.attribute_1 = temp;
        p_params.attribute_2 = 0;
        p_params.attribute_3 = 0;
        access_model_reliable_cancel(server.model_handle);
        generic_sensor_status_publish(&server, &p_params);
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published temp - Data: %d %d %d \n", p_params.attribute_1, p_params.attribute_2, p_params.attribute_3);
    sum = 0;
    count = 0;

}

void pwm_init()
{
NRF_PWM0->PSEL.OUT[1] = (RED_LED << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
NRF_PWM0->PSEL.OUT[2] = (GREEN_LED << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
NRF_PWM0->PSEL.OUT[3] = (BLUE_LED << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
NRF_PWM0->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
NRF_PWM0->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
NRF_PWM0->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_8 << PWM_PRESCALER_PRESCALER_Pos);
NRF_PWM0->COUNTERTOP  = (255 << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
NRF_PWM0->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
NRF_PWM0->DECODER     = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
NRF_PWM0->SEQ[0].PTR  = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
NRF_PWM0->SEQ[0].CNT  = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
NRF_PWM0->SEQ[0].REFRESH  = 0;
NRF_PWM0->SEQ[0].ENDDELAY = 0;
}

void set_pwm(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
      pwm_seq[0]=r; pwm_seq[1]=g; pwm_seq[2]=b; pwm_seq[3]=w;
      NRF_PWM0->TASKS_SEQSTART[0]=1;
}

void buzz_init()
{
    NRF_PWM1->PSEL.OUT[1] = (BUZZER << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM1->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM1->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM1->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM1->COUNTERTOP  = (267 << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
    NRF_PWM1->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    NRF_PWM1->DECODER     = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM1->SEQ[0].PTR  = ((uint32_t)(pwm_buzz) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM1->SEQ[0].CNT  = ((sizeof(pwm_buzz) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM1->SEQ[0].REFRESH  = 0;
    NRF_PWM1->SEQ[0].ENDDELAY = 0;
}

void set_buzz(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
      pwm_buzz[0]=r; pwm_buzz[1]=0x8000|g; pwm_buzz[2]=b; pwm_buzz[3]=w;
      NRF_PWM1->TASKS_SEQSTART[0]=1;
}

void speed_init()
{

NRF_PWM2->PSEL.OUT[1] = (FAN << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected <<PWM_PSEL_OUT_CONNECT_Pos);
NRF_PWM2->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
NRF_PWM2->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
NRF_PWM2->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_8 << PWM_PRESCALER_PRESCALER_Pos);
NRF_PWM2->COUNTERTOP  = (255 << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
NRF_PWM2->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
NRF_PWM2->DECODER     = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);               
NRF_PWM2->SEQ[0].PTR  = ((uint32_t)(pwm_speed) << PWM_SEQ_PTR_PTR_Pos);
NRF_PWM2->SEQ[0].CNT  = ((sizeof(pwm_speed) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
NRF_PWM2->SEQ[0].REFRESH  = 0;
NRF_PWM2->SEQ[0].ENDDELAY = 0;
}

void set_speed(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
      pwm_speed[0]=r; pwm_speed[1]=g ; pwm_speed[2]=b; pwm_speed[3]=w;
      NRF_PWM2->TASKS_SEQSTART[0]=1;
}

void wdt_init()
{
    uint32_t err_code=0;
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}


void restart()
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "shutdown %d\n", shutdown_time);
    shutdown_time--;
    if(!shutdown_time)
    {
       app_timer_stop(restart_timer);
      uint32_t retval = proxy_enable();
      //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enable Proxy %d \n", retval);
      if (retval == NRF_SUCCESS)
      {
          retval = proxy_start();
      }
      if (retval == NRF_SUCCESS)
      {
          retval = nrf_mesh_enable();
      }
      //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enable Mesh %d \n", retval);

      set_pwm(0,0,100,0);
      nrf_delay_ms(200);
      set_pwm(0,0,0,0);
    }
}

void triac_init()
{    
    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    ret_code_t err_code = nrf_drv_gpiote_out_init(TRIAC, &config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_task_enable(TRIAC);

}

void triac_on()
{
    ret_code_t err_code = nrfx_ppi_channel_alloc(&ppi_channel_timer_capture);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_channel_timer_capture, nrfx_gpiote_in_event_addr_get(ZC), nrfx_timer_task_address_get(&TIMER, NRF_TIMER_TASK_CAPTURE1));
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(ppi_channel_timer_capture, nrfx_timer_task_address_get(&TIMER, NRF_TIMER_TASK_CLEAR));
    err_code=nrfx_ppi_channel_enable(ppi_channel_timer_capture);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_channel1, nrfx_timer_event_address_get(&TIMER2, NRF_TIMER_EVENT_COMPARE0), nrfx_gpiote_set_task_addr_get(TRIAC));
    APP_ERROR_CHECK(err_code);

    nrfx_timer_resume(&TIMER);
    nrfx_gpiote_in_event_enable(ZC, true);
}

void relay_init()
{ 
    nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    ret_code_t err_code = nrfx_gpiote_out_init(RELAY, &out_config);

    nrfx_gpiote_out_task_enable(RELAY);
}

void ppi_relay_on_init()
{
    ret_code_t err_code = nrfx_ppi_channel_alloc(&ppi_channel_pin_high);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_channel_pin_high, nrfx_timer_event_address_get(&TIMER2, NRF_TIMER_EVENT_COMPARE0), nrfx_gpiote_set_task_addr_get(RELAY));
    APP_ERROR_CHECK(err_code);
}

void ppi_relay_off_init()
{
    ret_code_t err_code = nrfx_ppi_channel_alloc(&ppi_channel_pin_low);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_channel_pin_low, nrfx_timer_event_address_get(&TIMER2, NRF_TIMER_EVENT_COMPARE0), nrfx_gpiote_clr_task_addr_get(RELAY));
    APP_ERROR_CHECK(err_code);
}


fwid_t* mp_curr_fwid;

void dfu_info()
{
    bl_cmd_t fwid_cmd;
    fwid_cmd.type = BL_CMD_TYPE_INFO_GET;
    fwid_cmd.params.info.get.type = BL_INFO_TYPE_VERSION;
    fwid_cmd.params.info.get.p_entry = NULL;
    uint32_t error_code = nrf_mesh_dfu_cmd_send(&fwid_cmd);

    mp_curr_fwid = &fwid_cmd.params.info.get.p_entry->version;

    fw = mp_curr_fwid->app.app_version;
    app = mp_curr_fwid->app.app_id;
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"fw app %d %d\n", fw, app);
}

void mosfet_init()
{
   nrf_gpio_cfg_output(RELAY);
   nrf_gpio_pin_set(RELAY);

}

APP_TIMER_DEF(fan_timer);
static bool fan = false;

static void fan_timer_handler()
{
  fan = !fan;

  if(fan){ 
    set_speed(0, 250, 0, 0);     
     }
  else
    {
     set_speed(0,0,0,0);
    }
}


void sensor_on()
{

   nrf_gpio_cfg_output(SENSOR_ON);
   nrf_gpio_pin_set(SENSOR_ON);

}



int main(void)

{

    initialize();
    start();

    timer_3_init();
    timer_4_init();

    gpio_init();  
    
    pwm_init();

    buzz_init();

    speed_init();

    dfu_info();
       
    ret_code_t err_code;

    wdt_init();


    set_pwm(0,0,0,200);
    nrf_delay_ms(100);
    set_pwm(0,0,0,0);

    nrf_delay_ms(100);

    set_pwm(0,0,200,0);
    nrf_delay_ms(100);
    set_pwm(0,0,0,0);

    nrf_delay_ms(100);

    set_pwm(0,200,0,0);
    nrf_delay_ms(100);
    set_pwm(0,0,0,0);

    set_buzz(0, 125, 0, 0);
    nrf_delay_ms(100);
    set_buzz(0, 0, 0, 0);


    entry.record = 0x0003;
    mesh_config_entry_get(entry , &threshold);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published - Data: %d \n",threshold);

    entry.record = 0x0005;
    mesh_config_entry_get(entry , &Ro);
   if(Ro>0)
   {
   calibrated = true;
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,"  Ro:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(Ro));
   }

   entry.record = 0x0006;
   mesh_config_entry_get(entry, &send_time);
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor status published - Data: %d \n",send_time);

    mosfet_init();
    sensor_on();

    nrf_delay_ms(500);
    nrf_gpio_pin_set(RELAY);
    nrf_delay_ms(500);
    nrf_gpio_pin_clear(RELAY);

    APP_ERROR_CHECK(app_timer_create(&restart_timer, APP_TIMER_MODE_REPEATED, restart));
    //APP_ERROR_CHECK(app_timer_create(&encoder_timer, APP_TIMER_MODE_REPEATED, encoder_data));   
    APP_ERROR_CHECK(app_timer_create(&timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout)); 
    APP_ERROR_CHECK(app_timer_create(&device_timer, APP_TIMER_MODE_SINGLE_SHOT, device_status));
    APP_ERROR_CHECK(app_timer_create(&send_timer,APP_TIMER_MODE_REPEATED,send_timer_callback)); 
    APP_ERROR_CHECK(app_timer_create(&fan_timer, APP_TIMER_MODE_REPEATED,fan_timer_handler));
    start_adc();


   

    for (;;)
    {      
       nrf_drv_wdt_channel_feed(m_channel_id);
       nrf_delay_ms(500);
       //resistance();
         if(!m_device_provisioned)
        {
           set_pwm(0,0,200,200);
           nrf_delay_ms(300);
           set_pwm(0,0,0,0);
           nrf_delay_ms(200);
        }
        (void)sd_app_evt_wait();
    }

}
