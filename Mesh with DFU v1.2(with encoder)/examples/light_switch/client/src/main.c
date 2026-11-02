/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

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

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"
#include "health_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

/**************DFU*********************************/
#include <boards.h>
#include "nrf_mesh_events.h"
#include "log.h"
#include "nrf_mesh_dfu.h"
#include "app_util.h"
#include "nrf_mesh_serial.h"


/****************libuarte***************************/
#include "nrf_libuarte_async.h"
#include "nrf_drv_clock.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_queue.h"
#include "nrf_drv_gpiote.h"

#include "nrf_delay.h"
#include "app_scheduler.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define LEDS_MASK_DFU_RUNNING   (BSP_LED_0_MASK | BSP_LED_2_MASK)
#define LEDS_MASK_DFU_ENDED     (BSP_LED_0_MASK | BSP_LED_1_MASK)

#define APP_UNACK_MSG_REPEAT_COUNT   (1)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_DELAY_MS           (50)
/* Transition time value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_TRANSITION_TIME_MS (100)

APP_TIMER_DEF(health_timer);
APP_TIMER_DEF(encoder_timer);
APP_TIMER_DEF(timeout_timer);

#define HEALTH_TIME   30000
#define ALIVE_SCORE   10
#define TIMEOUT       180000

#define ENCODER       1000

#define PIN_IN1     17
#define PIN_IN2     19

#define BUTTON1  25
#define BUTTON2  24
#define BUTTON3  11  

#define TRIGGER_BUTTON_1 1
#define TRIGGER_BUTTON_2 2
#define TRIGGER_BUTTON_3 3
#define TRIGGER_BUTTON_4 4
#define TRIGGER_TEMPERATURE_SENSOR 5
#define TRIGGER_TIME 6

#define MAX_BRIGHTNESS 250
#define NIGHT_MODE 100

uint8_t brightness_limit=0;

bool flag=true;

static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event);
static health_client_t m_healths;



typedef struct
{
    uint8_t s_no;
    uint16_t node_address;
    uint16_t node_type;
    uint8_t node_state;
    uint16_t event;
    uint8_t trigger[11];
}event;


/******************TO save data in persistent memory*******************/

#define TRIGGER    0x0005
#define TRIGGER_START 0x0001

#define CONDITION  0x0006
#define CONDITION_START 0x000B

#define ACTION     0x0007
#define ACTION_START 0x0015

#define EVENT     0x0008
#define EVENT_START 0x0020

MESH_CONFIG_FILE(m_app_trigger_file, TRIGGER, MESH_CONFIG_STRATEGY_CONTINUOUS);
MESH_CONFIG_FILE(m_app_condition_file, CONDITION, MESH_CONFIG_STRATEGY_CONTINUOUS);
MESH_CONFIG_FILE(m_app_action_file, ACTION, MESH_CONFIG_STRATEGY_CONTINUOUS);

MESH_CONFIG_FILE(m_app_event_file, EVENT, MESH_CONFIG_STRATEGY_CONTINUOUS);


#define RECORD_START  0x0001

enum
{
	RECORD_1 = 0x0001,
	RECORD_2,
	RECORD_3,
	RECORD_4,
	RECORD_5,
};
//
#define APP_ENTRY_ID_1    MESH_CONFIG_ENTRY_ID(0x0010, RECORD_1)
#define APP_ENTRY_ID_2    MESH_CONFIG_ENTRY_ID(0x0010, RECORD_2)
#define APP_ENTRY_ID_3    MESH_CONFIG_ENTRY_ID(0x0010, RECORD_3)
#define APP_ENTRY_ID_4    MESH_CONFIG_ENTRY_ID(0x0010, RECORD_4)
#define APP_ENTRY_ID_5    MESH_CONFIG_ENTRY_ID(0x0010, RECORD_5)

#define APP_ENTRY_ID_TRIGGER      MESH_CONFIG_ENTRY_ID(TRIGGER, TRIGGER_START)
#define APP_ENTRY_ID_CONDITION    MESH_CONFIG_ENTRY_ID(CONDITION, CONDITION_START)
#define APP_ENTRY_ID_ACTION       MESH_CONFIG_ENTRY_ID(ACTION, ACTION_START)

#define APP_ENTRY_ID_EVENT       MESH_CONFIG_ENTRY_ID(EVENT, EVENT_START)

/* Live RAM representation of the value */
static uint64_t m_live_value[30] ={0};
static uint32_t app_entry_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const uint64_t * p_value = (const uint64_t *) p_entry;
    m_live_value[id.record] = *p_value;
    return NRF_SUCCESS;
}
static void app_entry_getter(mesh_config_entry_id_t id, void * p_entry)
{
    uint64_t * p_value = (uint64_t *) p_entry;
    *p_value = m_live_value[id.record];
}




static event value[10] ={0};
static uint32_t app_event_entry_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    const event * p_value = (const event *) p_entry;
    value[id.record] = *p_value;
    return NRF_SUCCESS;
}

static void app_event_entry_getter(mesh_config_entry_id_t id, void * p_entry)
{
    event * p_value = (event *) p_entry;
    *p_value = value[id.record];
}


union
{
    uint64_t set;
    struct 
    {
        uint8_t trigger;
        uint8_t condition_id[3];
        uint8_t action_id[3];
        uint8_t trigger_state;
    }data;
}trigger, check_trigger;


union
{
    uint64_t set;
    event_manager events;
}event_handler[4];


union
{
   uint64_t set;
    struct 
    {
        uint8_t condition;
        uint8_t attribute_1;
        uint8_t attribute_2;
        uint8_t attribute_3;
    }data;
}condition, check_condition;

union
{
    uint64_t set;
    struct 
    {
        uint8_t client;
        uint8_t command;
        uint8_t data[6];
    }data;
}action, check_action;

MESH_CONFIG_ENTRY(m_app_entry_trigger,
                  APP_ENTRY_ID_TRIGGER,
                  10, // The entry is singular
                  sizeof(uint64_t),
                  app_entry_setter,
                  app_entry_getter,
                  NULL, // No need for a delete callback
                  false); // There is a default value


MESH_CONFIG_ENTRY(m_app_entry_condition,
                  APP_ENTRY_ID_CONDITION,
                  10, // The entry is singular
                  sizeof(uint64_t),
                  app_entry_setter,
                  app_entry_getter,
                  NULL, // No need for a delete callback
                  false); // There is a default value


MESH_CONFIG_ENTRY(m_app_entry_action,
                  APP_ENTRY_ID_ACTION,
                  10, // The entry is singular
                  sizeof(uint64_t),
                  app_entry_setter,
                  app_entry_getter,
                  NULL, // No need for a delete callback
                  false); // There is a default value


MESH_CONFIG_ENTRY(m_app_entry_event,
                  APP_ENTRY_ID_EVENT,
                  10, // The entry is singular
                  sizeof(event),
                  app_event_entry_setter,
                  app_event_entry_getter,
                  NULL, // No need for a delete callback
                  false); // There is a default value


//MESH_CONFIG_ENTRY(m_app_entry_2,
//                  APP_ENTRY_ID_2,
//                  1, // The entry is singular
//                  sizeof(uint32_t),
//                  app_entry_setter,
//                  app_entry_getter,
//                  NULL, // No need for a delete callback
//                  true); // There is a default value
//
//MESH_CONFIG_ENTRY(m_app_entry_3,
//                  APP_ENTRY_ID_3,
//                  1, // The entry is singular
//                  sizeof(uint32_t),
//                  app_entry_setter,
//                  app_entry_getter,
//                  NULL, // No need for a delete callback
//                  true); // There is a default value
//
//MESH_CONFIG_ENTRY(m_app_entry_4,
//                  APP_ENTRY_ID_4,
//                  1, // The entry is singular
//                  sizeof(uint32_t),
//                  app_entry_setter,
//                  app_entry_getter,
//                  NULL, // No need for a delete callback
//                  true); // There is a default value

//MESH_CONFIG_ENTRY(m_app_entry_5,
//                  APP_ENTRY_ID_5,
//                  1, // The entry is singular
//                  sizeof(uint32_t),
//                  app_entry_setter,
//                  app_entry_getter,
//                  NULL, // No need for a delete callback
//                  true); // There is a default value


union
{
   uint32_t data;
   struct
   {
       uint8_t state_1;
       uint8_t state_2;
       uint8_t state_3;
       uint8_t state_4;
   }device;
}control,control1;
/******************************************************************************************/



/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_onoff_status_params_t * p_in);
static void app_generic_encoder_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_encoder_status_params_t * p_in);
static void app_generic_button_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_button_status_params_t * p_in);
static void app_generic_sensor_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_sensor_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);


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
                hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
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
//            nrf_gpio_pin_clear(13);
            hal_led_mask_set(LEDS_MASK_DFU_RUNNING, true);
            break;

        case NRF_MESH_EVT_DFU_END:
//            nrf_gpio_pin_clear(14);
            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            hal_led_mask_set(LEDS_MASK_DFU_ENDED, true);
            break;

        case NRF_MESH_EVT_DFU_BANK_AVAILABLE:
//            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            ERROR_CHECK(nrf_mesh_dfu_bank_flash(p_evt->params.dfu.bank.transfer.dfu_type));
            break;

        default:
            break;

    }
}


/*****************************************************************************
 * Static variables
 *****************************************************************************/
static generic_onoff_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .encoder_status_cb = app_generic_encoder_client_status_cb,
    .button_status_cb = app_generic_button_client_status_cb,
    .sensor_status_cb = app_generic_sensor_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
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
    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}


uint8_t current_pos=0;
struct element
{
   uint16_t address;
   uint16_t model_handle;
   uint8_t score;
   bool alive;
   bool active;
}item[CLIENT_MODEL_INSTANCE_COUNT];

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
  
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            flag=true;
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success. Score %d %d\n", item[model_handle].score, model_handle);
            if(item[model_handle].model_handle==model_handle)
            {
               item[model_handle].alive=true;
               item[model_handle].active=false;
               if(item[model_handle].score<10)
                ++item[model_handle].score;
            }
            else
            {
               dsm_handle_t publication_address=DSM_HANDLE_INVALID;
               uint8_t valid;
               valid=access_model_publish_address_get(model_handle, &publication_address);
               if(valid==NRF_SUCCESS)
               {
                 item[model_handle].model_handle = model_handle;
                 item[model_handle].address = publication_address;
                 item[model_handle].alive=true;
                 item[model_handle].active=false;
                 item[model_handle].score=ALIVE_SCORE;
               }
            }
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            flag=true;
            if(item[model_handle].score>0)
            --item[model_handle].score;
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            flag=false;
            if(item[model_handle].score>0)
            --item[model_handle].score;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

uint8_t new_value=0;
uint8_t server=0, button=0, lights_counter=0;
bool button1=0, button2=0, button3=0, button4=0;

/* 
        Callback function to receive onoff(Nordic API) data from Switchplate
        data received=>
                   p_in->step =>      encoder steps
                   p_in->incr_decr => increment(1) OR decrement(0)
 */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_onoff_status_params_t * p_in)
{                                    
   //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x \n", p_meta->src.value);
}

bool trigger_condition(uint8_t);
/*
    Callback function to receive encoder data from Switchplate
    data received=>
                   p_in->step =>      encoder steps
                   p_in->incr_decr => increment(1) OR decrement(0)
*/
static void app_generic_encoder_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_encoder_status_params_t * p_in)
{
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Encoder data %d %d\n", p_in->incr_decr, p_in->step);

              uint32_t status = NRF_SUCCESS;
              generic_onoff_set_params_t set_params;
              generic_rgbw_set_params_t rgbw_set_params;
              model_transition_t transition_params;

              transition_params.delay_ms = APP_ONOFF_DELAY_MS;
              transition_params.transition_time_ms = APP_ONOFF_TRANSITION_TIME_MS;

              static uint8_t tid = 0;
			 
	      set_params.tid = tid;
              rgbw_set_params.tid=tid++;

                  if(button==8)
                  {
                     if(p_in->incr_decr)
                     {
                          if(lights_counter==1)
                          {
                             if((control.device.state_1 + p_in->step)<255)
                             {
                                 control.device.state_1+= p_in->step;
                             }
                             else
                             {
                                 control.device.state_1=255; 
                             }
                             new_value = control.device.state_1;
                          }
                          else if(lights_counter==2)
                          {
                             if((control.device.state_2 + p_in->step)<255)
                             {
                                control.device.state_2+= p_in->step;
                             }
                             else
                             {
                                 control.device.state_2=255; 
                             }
                             new_value = control.device.state_2;
                          }
                          else if(lights_counter==3)
                          {
                             if((control.device.state_3 + p_in->step)<255)
                             {
                                control.device.state_3+= p_in->step;
                             }
                             else
                             {
                                 control.device.state_3=255; 
                             }
                             new_value = control.device.state_3;
                          }
                          else if(lights_counter==4)
                          {
                             if((control.device.state_4 + p_in->step)<255)
                             {
                                control.device.state_4+= p_in->step;
                             }
                             else
                             {
                                 control.device.state_4=255; 
                             }
                             new_value = control.device.state_4;
                          }
                     }
                     else
                     {
                        if(lights_counter==1)
                        {
                             if((control.device.state_1 - p_in->step)>5)
                             {
                                 control.device.state_1-= p_in->step;
                             }
                             else
                             {
                                 control.device.state_1=5; 
                             }
                              new_value = control.device.state_1;
                        }
                        else if(lights_counter==2)
                        {
                             if((control.device.state_2 - p_in->step)>5)
                             {
                                control.device.state_2-= p_in->step;
                             }
                             else
                             {
                                 control.device.state_2=5; 
                             }
                             new_value = control.device.state_2;
                        }
                        else if(lights_counter==3)
                        {
                             if((control.device.state_3 - p_in->step)>5)
                             {
                                control.device.state_3-= p_in->step;
                             }
                             else
                             {
                                 control.device.state_3=5; 
                             }
                              new_value = control.device.state_3;
                        }
                        else if(lights_counter==4)
                        {
                             if((control.device.state_4 - p_in->step)>5)
                             {
                                control.device.state_4-= p_in->step;
                             }
                             else
                             {
                                 control.device.state_4=5; 
                             }
                           new_value = control.device.state_4;
                        }
                     }
                 }
                 else
                 {
                     if(p_in->incr_decr)
                     {
                         if((control.device.state_1 + p_in->step)<255)
                         {
                             control.device.state_1+= p_in->step;
                         }
                         else
                         {
                             control.device.state_1=255; 
                         }
                         if((control.device.state_2 + p_in->step)<255)
                         {
                            control.device.state_2+= p_in->step;
                         }
                         else
                         {
                             control.device.state_2=255; 
                         }
                         if((control.device.state_3 + p_in->step)<255)
                         {
                            control.device.state_3+= p_in->step;
                         }
                         else
                         {
                             control.device.state_3=255; 
                         }
                         if((control.device.state_4 + p_in->step)<255)
                         {
                            control.device.state_4+= p_in->step;
                         }
                         else
                         {
                             control.device.state_4=255; 
                         }
                     }
                     else
                     {
                         if((control.device.state_1 - p_in->step)>5)
                         {
                             control.device.state_1-= p_in->step;
                         }
                         else
                         {
                             control.device.state_1=5; 
                         }
                         if((control.device.state_2 - p_in->step)>5)
                         {
                            control.device.state_2-= p_in->step;
                         }
                         else
                         {
                             control.device.state_2=5; 
                         }
                         if((control.device.state_3 - p_in->step)>5)
                         {
                            control.device.state_3-= p_in->step;
                         }
                         else
                         {
                             control.device.state_3=5; 
                         }
                         if((control.device.state_4 - p_in->step)>5)
                         {
                            control.device.state_4-= p_in->step;
                         }
                         else
                         {
                             control.device.state_4=5; 
                         }
                     }
                 }
						  
	              rgbw_set_params.red = control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                    
                    if(button==4)
                      transition_params.delay_ms = 10;
                    else 
                      transition_params.delay_ms = 5;

                     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "record 2 -> %d %d %d %d\n", control.device.state_1, control.device.state_2, control.device.state_3, control.device.state_4);
                
                    
                    if(item[m_clients[server].model_handle].alive && server)
                    { 
                      (void)access_model_reliable_cancel(m_clients[server].model_handle);
                      status = generic_rgbw_client_set(&m_clients[server], &rgbw_set_params, &transition_params);
                    }
                    if(item[m_clients[0].model_handle].alive && server)
                    {

                         if(button!=8 || button==4)      //if setting mode is not active then send the largest value otherwise send that praticular lights value
                         {
                             
                             set_params.on_off = control.device.state_1>control.device.state_2?(control.device.state_1>control.device.state_3?control.device.state_1:control.device.state_3):(control.device.state_2>control.device.state_3?control.device.state_2:control.device.state_3);
                             set_params.on_off = control.device.state_4>set_params.on_off?control.device.state_4:set_params.on_off;

                             new_value=set_params.on_off;
                         }

//                            (void)access_model_reliable_cancel(m_clients[0].model_handle);
//                            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);

                         app_timer_stop(encoder_timer);                                               
                         app_timer_start(encoder_timer, APP_TIMER_TICKS(ENCODER), NULL); 
                        
                    }
}

bool night_mode=0;
uint8_t mode=0;

static uint8_t tid = 0;
/*
    Scheduler handler function to send RGBW messages
*/
void rgbw_scheduler_handler(void *p_event_data, uint16_t size)
{ //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RGBW scheduler\n");
     uint32_t status = NRF_SUCCESS;
     generic_rgbw_set_params_t rgbw_set_params = *((generic_rgbw_set_params_t *) p_event_data);

     model_transition_t transition_params;

     transition_params.delay_ms = APP_ONOFF_DELAY_MS;
     transition_params.transition_time_ms = APP_ONOFF_TRANSITION_TIME_MS;

     

      if(item[m_clients[1].model_handle].alive && item[m_clients[1].model_handle].active==false)
      {//__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RGBW schedule executed\n");
          __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Red %d\n", rgbw_set_params.red);

          tid++;

           rgbw_set_params.tid=tid;
           rgbw_set_params.red=tid;

          item[m_clients[1].model_handle].active=true;
          (void)access_model_reliable_cancel(m_clients[1].model_handle);
           status = generic_rgbw_client_set(&m_clients[1], &rgbw_set_params, &transition_params);
      }
      else
      {//__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RGBW rescheduled\n");
         app_sched_event_put(&rgbw_set_params, sizeof(rgbw_set_params), rgbw_scheduler_handler);
      }
}


/*
   Callback function to receive button press data from Switchplate
   data received =>
                     p_in->b_number => button pressed
                     p_in->action   => here we can send modes of the button
*/

static void app_generic_button_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_button_status_params_t * p_in)
{
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "button data 0x%04x %d %d\n", p_rx_msg->meta_data.src.value, p_in->b_number, p_in->action);

                 uint32_t status = NRF_SUCCESS;
              generic_onoff_set_params_t set_params;
              generic_rgbw_set_params_t rgbw_set_params;
              generic_button_response_params_t button_response;
              model_transition_t transition_params;

              transition_params.delay_ms = APP_ONOFF_DELAY_MS;
              transition_params.transition_time_ms = APP_ONOFF_TRANSITION_TIME_MS;

              tid++;
			 
	      set_params.tid = tid;
              rgbw_set_params.tid=tid;

   switch(p_in->b_number)
   {
        case 1: // Bathroom
                button1=!button1;
                
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BUTTON 1 pressed %d\n", button1);
                           if(button1)
                           {//1
                              rgbw_set_params.red = tid; rgbw_set_params.green = 0; rgbw_set_params.blue = 0; rgbw_set_params.white = 0;
                           }
                           else
                           {
                              if(night_mode)
                              {//0
                                 rgbw_set_params.red = tid; rgbw_set_params.green = 1; rgbw_set_params.blue = 0; rgbw_set_params.white = 0;
                              }
                              else
                              {//0
                                 rgbw_set_params.red = tid; rgbw_set_params.green = 0; rgbw_set_params.blue = 0; rgbw_set_params.white = 0;
                              }
                           }
                          
                          transition_params.delay_ms = 5;
                          transition_params.transition_time_ms = button1;
				
                          if(item[m_clients[1].model_handle].alive && item[m_clients[1].model_handle].active==false)
                          {
                              __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Red %d\n", tid);
                              item[m_clients[1].model_handle].active=true;
                              (void)access_model_reliable_cancel(m_clients[1].model_handle);
                               status = generic_rgbw_client_set(&m_clients[1], &rgbw_set_params, &transition_params);
                          }
                          else
                          {//__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RGBW scheduled\n");
                             app_sched_event_put(&rgbw_set_params, sizeof(rgbw_set_params), rgbw_scheduler_handler);
                          }

                          if(!button1)
                          {
                              app_timer_stop(timeout_timer);                                               
                              app_timer_start(timeout_timer, APP_TIMER_TICKS(TIMEOUT), NULL); 
                          }


                          transition_params.delay_ms = 25;
                          if(item[m_clients[0].model_handle].alive && item[m_clients[0].model_handle].active==false)
                          {   // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SwitchPlate data sent\n");
                               button_response.button = 1;
                               button_response.status = button1;
                               button_response.tid = tid;
                               item[m_clients[0].model_handle].active=true;
                               (void)access_model_reliable_cancel(m_clients[0].model_handle);
                               status = generic_button_response_set(&m_clients[0], &button_response, &transition_params);
                          }
                               button=1;
               break;
        case 2: // Dressing
                button2=!button2;
                            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BUTTON 2 pressed %d\n", button2);

                            set_params.on_off=0;
                            transition_params.delay_ms = 5;

                            if(p_in->action==1)     //Action => on_off
                            {
                               if(mode==0)
                               {
                                 if(button2)
                                 {
                                    lights_counter=0;
                                    button=2;
                                    mesh_config_entry_get(APP_ENTRY_ID_2, &control.data);

                                    if(night_mode)
                                    {
                                          if(control.device.state_1>brightness_limit)
                                          {
                                              control.device.state_1=brightness_limit;
                                          }

                                          if(control.device.state_2>brightness_limit)
                                          {
                                              control.device.state_2=brightness_limit;
                                          }

                                          if(control.device.state_3>brightness_limit)
                                          {
                                              control.device.state_3=brightness_limit;
                                          }

                                          if(control.device.state_4>brightness_limit)
                                          {
                                              control.device.state_4=brightness_limit;
                                          }
                                    }
//                                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "record 2 -> %d %d %d %d\n", control.device.state_1, control.device.state_2, control.device.state_3, control.device.state_4);
                                 }
                                 else
                                 {
                                     control.device.state_1 = 0; control.device.state_2= 0; control.device.state_3= 0; control.device.state_4= 0;
                                 }

                                 rgbw_set_params.red= control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                                 transition_params.transition_time_ms = button2;

                                       set_params.on_off = control.device.state_1>control.device.state_2?(control.device.state_1>control.device.state_3?control.device.state_1:control.device.state_3):(control.device.state_2>control.device.state_3?control.device.state_2:control.device.state_3);
                                       set_params.on_off = control.device.state_4>set_params.on_off?control.device.state_4:set_params.on_off;

                                        if(item[m_clients[2].model_handle].alive)
                                        {
                                            (void)access_model_reliable_cancel(m_clients[2].model_handle);
                                            status = generic_rgbw_client_set(&m_clients[2], &rgbw_set_params, &transition_params);
                                        }

                                        if(item[m_clients[0].model_handle].alive)
                                        {
                                             button_response.button = 2;
                                             button_response.status = button2;
                                             button_response.tid = tid;
                                             (void)access_model_reliable_cancel(m_clients[0].model_handle);
                                             status = generic_button_response_set(&m_clients[0], &button_response, &transition_params);
                                        }
                               }
                               else if(mode==1)  //  Setting mode
                               {
                                     ++lights_counter;

                                    switch(lights_counter)
                                    {
                                       case 1:  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light 1 %d\n", new_value);           //toggle the light which is selected and only change the brigthness of that light.
                                                set_params.on_off = control.device.state_1;
                                                transition_params.transition_time_ms = 1;
                                              break;
                                       case 2:  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light 2 %d\n", new_value);
                                                  set_params.on_off = control.device.state_2;
                                                transition_params.transition_time_ms = 2;
                                              break;
                                       case 3:  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light 3 %d\n", new_value);
                                                set_params.on_off = control.device.state_3;
                                                transition_params.transition_time_ms = 3;
                                              break;
                                       case 4:  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light 4 %d\n", new_value);
                                                set_params.on_off = control.device.state_4;
                                                transition_params.transition_time_ms = 4;
                                              break;
                                    }
//                                   rgbw_set_params.red= control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                                   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light %d\n", lights_counter);

                                   if(item[m_clients[2].model_handle].alive)
                                   {                            
                                      (void)access_model_reliable_cancel(m_clients[2].model_handle);
                                      status = generic_onoff_client_set(&m_clients[2], &set_params, &transition_params);
                                   }
                                   
                               }
                                       
                              
                            }
                            else if(p_in->action==2)     // Action => save mode
                            {
                                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BUTTON 2 save mode %d %d %d %d\n", control.device.state_1, control.device.state_2, control.device.state_3, control.device.state_4);
                               
                                if(!button2)
                                {
                                    mesh_config_entry_set(APP_ENTRY_ID_2, &control.data);
                                    mode=0;
                                }
                                else
                                {
                                    mode=1;
                                }

//                               set_params.on_off=0;
//                               if(item[m_clients[0].model_handle].alive)
//                               {                            
//                                  (void)access_model_reliable_cancel(m_clients[0].model_handle);
//                                  status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
//                               }

                               button=0;
                            }
               break;
        case 3: // Night mode
                button3=!button3;
                night_mode = button3;

                         if(button2)
                         {
                              mesh_config_entry_get(APP_ENTRY_ID_2, &control.data); 
                            
                              rgbw_set_params.red = control.device.state_1>brightness_limit?brightness_limit:control.device.state_1; 
                              rgbw_set_params.green= control.device.state_2>brightness_limit?brightness_limit:control.device.state_2; 
                              rgbw_set_params.blue= control.device.state_3>brightness_limit?brightness_limit:control.device.state_3; 
                              rgbw_set_params.white= control.device.state_4>brightness_limit?brightness_limit:control.device.state_4;
                              

                              control.device.state_1 = rgbw_set_params.red; 
                              control.device.state_2 = rgbw_set_params.green; 
                              control.device.state_3 = rgbw_set_params.blue; 
                              control.device.state_4 = rgbw_set_params.white; 

                              transition_params.delay_ms=5;
                              transition_params.transition_time_ms = 15;
				
                              if(item[m_clients[2].model_handle].alive)
                              {
                                  (void)access_model_reliable_cancel(m_clients[2].model_handle);
                                  generic_rgbw_client_set(&m_clients[2], &rgbw_set_params, &transition_params);
                              }

                              if(item[m_clients[0].model_handle].alive)
                              {
                                   button_response.button = 3;
                                   button_response.status = button3;
                                   button_response.tid = tid;
                                   (void)access_model_reliable_cancel(m_clients[0].model_handle);
                                   status = generic_button_response_set(&m_clients[0], &button_response, &transition_params);
                              }
                         }

                         if(button4)
                         {
                            
                            mesh_config_entry_get(APP_ENTRY_ID_4, &control.data); 
                            
                            rgbw_set_params.red = control.device.state_1>brightness_limit?brightness_limit:control.device.state_1; 
                            rgbw_set_params.green= control.device.state_2>brightness_limit?brightness_limit:control.device.state_2; 
                            rgbw_set_params.blue= control.device.state_3>brightness_limit?brightness_limit:control.device.state_3; 
                            rgbw_set_params.white= control.device.state_4>brightness_limit?brightness_limit:control.device.state_4;

                              control.device.state_1 = rgbw_set_params.red; 
                              control.device.state_2 = rgbw_set_params.green; 
                              control.device.state_3 = rgbw_set_params.blue; 
                              control.device.state_4 = rgbw_set_params.white;

                            transition_params.delay_ms=6;
                            transition_params.transition_time_ms = 15;
				
                            if(item[m_clients[4].model_handle].alive)
                            {
                                (void)access_model_reliable_cancel(m_clients[4].model_handle);
                                 generic_rgbw_client_set(&m_clients[4], &rgbw_set_params, &transition_params);
                            }

                            if(item[m_clients[0].model_handle].alive)
                            {
                                 button_response.button = 3;
                                 button_response.status = button3;
                                 button_response.tid = tid;
                                 (void)access_model_reliable_cancel(m_clients[0].model_handle);
                                 status = generic_button_response_set(&m_clients[0], &button_response, &transition_params);
                            }
                         }
                        

                           set_params.tid=++tid;
                           transition_params.delay_ms=15;
                           if(item[m_clients[4].model_handle].alive)            //brightness limit for cupboard strips
                           {
                                    set_params.on_off=brightness_limit;
                                   (void)access_model_reliable_cancel(m_clients[4].model_handle);
                                    generic_onoff_client_set(&m_clients[4], &set_params, &transition_params);
                           }
               break;
        case 4: // Cupboard
                button4=!button4;

                             __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "BUTTON 4 pressed %d\n", button4);
                        if(p_in->action==1)
                        {
                           if(button4)
                           {
                              mesh_config_entry_get(APP_ENTRY_ID_4, &control.data); 
//                              rgbw_set_params.red = control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                              
                              if(control.device.state_1>brightness_limit)
                              {
                                  control.device.state_1=brightness_limit;
                              }

                              if(control.device.state_2>brightness_limit)
                              {
                                  control.device.state_2=brightness_limit;
                              }

                              if(control.device.state_3>brightness_limit)
                              {
                                  control.device.state_3=brightness_limit;
                              }

                              if(control.device.state_4>brightness_limit)
                              {
                                  control.device.state_4=brightness_limit;
                              }

                              button=4;
                           
                           }
                           else
                           {
                              control.device.state_1 = 0; control.device.state_2= 0; control.device.state_3= 0; control.device.state_4= 0;
                              button=0;
                           } 
                
                           
                            rgbw_set_params.red= control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                            transition_params.delay_ms=6;
                            transition_params.transition_time_ms = button4;
				
                            if(item[m_clients[4].model_handle].alive)
                            {
                                (void)access_model_reliable_cancel(m_clients[4].model_handle);
                                status = generic_rgbw_client_set(&m_clients[4], &rgbw_set_params, &transition_params);
                            }

                            if(item[m_clients[0].model_handle].alive)
                            {
                                 button_response.button = 4;
                                 button_response.status = button4;
                                 button_response.tid = tid;
                                 (void)access_model_reliable_cancel(m_clients[0].model_handle);
                                 status = generic_button_response_set(&m_clients[0], &button_response, &transition_params);
                            }
                              
                       }
                       else if(p_in->action==2)    //save values for button 4
                       { 
                            if(new_value!=0)
                            {
                              control.device.state_1 = new_value; control.device.state_2 = new_value; control.device.state_3 = new_value; control.device.state_4 = new_value;
                             
                              rgbw_set_params.red= control.device.state_1; rgbw_set_params.green= control.device.state_2; rgbw_set_params.blue= control.device.state_3; rgbw_set_params.white= control.device.state_4;
                              transition_params.delay_ms=5;
                              transition_params.transition_time_ms = button4;
                             
                              mesh_config_entry_set(APP_ENTRY_ID_4, &control.data);
                            }
                            button=0;
                       }
               break;
   }
 
}

/*
    Callback function to receive sensor data from various sensor's
    data received =>
                     p_in->attribute_1 => first sensor value
                     p_in->attribute_2 => second sensor value
                     p_in->attribute_3 => third sensor value

*/
static void app_generic_sensor_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_t * p_rx_msg,
                                               const generic_sensor_status_params_t * p_in)
{
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor data %d %d %d\n", p_in->attribute_1, p_in->attribute_2, p_in->attribute_3);
}

/*
   Node reset function 
   Can be called only by the provisioner
*/
static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

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
    "\t\t------------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) Send a message to the odd group (address: 0xC003) to turn on LED 1.\n"
    "\t\t Button/RTT 2) Send a message to the odd group (address: 0xC003) to turn off LED 1.\n"
    "\t\t Button/RTT 3) Send a message to the even group (address: 0xC002) to turn on LED 1.\n"
    "\t\t Button/RTT 4) Send a message to the even group (address: 0xC002) to turn off LED 1.\n"
    "\t\t------------------------------------------------------------------------------------\n";
#endif

//uint8_t last=0,current=0,counter=0;
//
//uint8_t i = 0, s=0;
//bool menu=false, trig=false, cond=false, act=false, edit=false, save=false;

/*
    Button event handler by nordic
*/
void button_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t ac)
{
    if(pin==BUTTON1)
    {

    }
    else if(pin==BUTTON2)
    {

    }
    else if(pin==BUTTON3)
    {

    }
}

uint8_t serial[20]={0};

/*
   Function to view RTT data
*/
static void rtt_input_handler(int key)
{

}

/*
   Function to find if the particular address is present in the health monitor
*/

void find_item(uint16_t address)
{
   for(uint8_t i=0; i<CLIENT_MODEL_INSTANCE_COUNT; i++)             //limit the loop to the number of clients in use(not done)     
   {
       if(item[i].address==address)
       {//__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Item found \n" );
           item[i].alive=true;
           item[i].score=ALIVE_SCORE;
       }
   }
}

/*****************************************************************************/
/**** Health Model related callbacks ****/
static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    switch (p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
            __LOG(LOG_SRC_APP,
                  LOG_LEVEL_INFO,
                  "Node 0x%04x alive with %u active fault(s), RSSI: %d\n",
                  p_event->p_meta_data->src.value,
                  p_event->data.fault_status.fault_array_length,
                  ((p_event->p_meta_data->p_core_metadata->source == NRF_MESH_RX_SOURCE_SCANNER)
                       ? p_event->p_meta_data->p_core_metadata->params.scanner.rssi
                       : 0));

                       find_item(p_event->p_meta_data->src.value);
            break;
        default:
            break;
    }
}

/*
   Callback function to initialize client model and health monitor
*/
static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

   ERROR_CHECK(health_client_init(&m_healths, 0, app_health_event_cb));

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
        
        item[i+1].alive=true;
        item[i+1].active=false;
        item[i+1].score=ALIVE_SCORE;

        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
    }
}

/*
     Function to initialize mesh parameters
*/
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

#if NRF_MESH_SERIAL_ENABLE
    ERROR_CHECK(nrf_mesh_serial_init(NULL));
#endif

    m_evt_handler.evt_cb = mesh_evt_handler;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

/*
    Initialize timer, ble stack, gap parameters, conncetion parameters and mesh
*/
static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

/*
    Start Mesh Stack 
*/
static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

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
            .p_device_uri = EX_URI_LS_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }


#if NRF_MESH_SERIAL_ENABLE
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling serial interface...\n");
    ERROR_CHECK(nrf_mesh_serial_enable());
#endif

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

/*
    Function to monitor if any node is dead or alive
    decides alive status on the bases of message transfer timeout's
*/
static void health_monitor()
{
      for(uint8_t i=0;i<CLIENT_MODEL_INSTANCE_COUNT;i++)                       //limit the loop to the number of clients in use  
      {
          if(item[i].score<=0)
          {
             item[i].alive=false;
          }
      }
}

/*
    Libuarte define 
    parameter_1          Instance name.
    parameter_2        UARTE instance used.
    parameter_3       TIMER instance used by libuarte for bytes counting.
    parameter_4       RTC instance used for timeout. If set to NRF_LIBUARTE_PERIPHERAL_NOT_USED
                      then TIMER instance is used or app_timer instance if _timer1_idx is also set
                      to NRF_LIBUARTE_PERIPHERAL_NOT_USED.
    parameter_5       TIMER instance used for timeout. If set to NRF_LIBUARTE_PERIPHERAL_NOT_USED
                      then RTC instance is used or app_timer instance if _rtc1_idx is also set
                      to NRF_LIBUARTE_PERIPHERAL_NOT_USED.
    parameter_6       Size of single RX buffer. Size impacts accepted latency between buffer
                      request and providing next buffer. Next must be provided within before
                      _rx_buf_size bytes is received.
    parameter_7       Number of buffers in the RX buffer pool. Size impacts accepted latency
                      between NRF_LIBUARTE_ASYNC_EVT_RX_DATA event and
                      @ref nrf_libuarte_async_rx_free.
*/
NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 1, 1, 1, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 3);

static uint8_t text[] = "Hello\n";
static uint8_t text_size = sizeof(text);
static volatile bool m_loopback_phase;

/*
    Uart buffer
*/
typedef struct {
    uint8_t * p_data;
    uint32_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);


/*
  Uart event handler to receive uart data 
*/

void uart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t ret;
// nrf_gpio_pin_toggle(13);
    switch (p_evt->type)
    {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
            //nrf_gpio_pin_clear(13);
            break;
        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
//            nrf_gpio_pin_clear(15);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data recevied %s\n", p_evt->data.rxtx.p_data);
//            button_event_handler(1);
            nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);    
            //m_loopback_phase = false;
            break;
        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
//             nrf_gpio_pin_clear(14);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data sent %s\n", p_evt->data.rxtx.p_data);
            break;
        default:
            break;
    }
}


bool flag1, flag2, dir;

/*
    GPIO interrupt handler function was used for encoder interrupt
*/
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//  encoder(nrf_gpio_pin_read(PIN_IN1), nrf_gpio_pin_read(PIN_IN2));
}

/*
    GPIO interrupt handler function was used for encoder interrupt
*/
void in_pin_handler2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
 // encoder(nrf_gpio_pin_read(PIN_IN1), nrf_gpio_pin_read(PIN_IN2));
}


/*
   Function to intialize GPIO
*/

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(PIN_IN1, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN1, true);

    nrf_drv_gpiote_in_config_t in_config2 = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config2.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(PIN_IN2, &in_config2, in_pin_handler2);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN2, true);

    nrf_drv_gpiote_in_config_t in_config3 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config3.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON1, &in_config3, button_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON1, true);

    nrf_drv_gpiote_in_config_t in_config4 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config4.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON2, &in_config4, button_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON2, true);

    nrf_drv_gpiote_in_config_t in_config5 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config5.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON3, &in_config5, button_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON3, true);
}

/*
    Timer handler was using it to send encoder messages
*/
void encoder_data()
{
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;

    set_params.on_off=new_value;
    set_params.tid=new_value;
    transition_params.delay_ms=10;
    transition_params.transition_time_ms=10;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data %d\n", new_value);

      (void)access_model_reliable_cancel(m_clients[0].model_handle);
      generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);

}


void timeout()
{
       model_transition_t transition_params;

//      generic_rgbw_set_params_t rgbw_set_params;
//     
//
//      rgbw_set_params.tid=6;
//
//      transition_params.delay_ms = 6;
//      transition_params.transition_time_ms = 0;
//          
//      if(item[m_clients[1].model_handle].alive)
//      {
//          (void)access_model_reliable_cancel(m_clients[1].model_handle);
//          generic_rgbw_client_set(&m_clients[1], &rgbw_set_params, &transition_params);
//      }
}

//void clock_event_handler(nrfx_clock_evt_type_t event) {};

#define SCHED_MAX_EVENT_DATA_SIZE sizeof(generic_rgbw_set_params_t)+4
#define SCHED_QUEUE_SIZE    10

int main(void)
{

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    initialize();
    start();
    
    gpio_init();

        nrf_drv_clock_lfclk_request(NULL);
        ret_code_t err_code;


    nrf_libuarte_async_config_t nrf_libuarte_async_config = {
            .tx_pin     = 22,
            .rx_pin     = 23,
            .baudrate   = NRF_UARTE_BAUDRATE_115200,
            .parity     = NRF_UARTE_PARITY_EXCLUDED,
            .hwfc       = NRF_UARTE_HWFC_DISABLED,
            .timeout_us = 100,
            .int_prio   = APP_IRQ_PRIORITY_LOW
    };

    err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&libuarte);
    APP_ERROR_CHECK(err_code);

    nrf_libuarte_async_enable(&libuarte);


    APP_ERROR_CHECK(app_timer_create(&health_timer, APP_TIMER_MODE_REPEATED, health_monitor));
    APP_ERROR_CHECK(app_timer_create(&encoder_timer, APP_TIMER_MODE_SINGLE_SHOT, encoder_data));
    APP_ERROR_CHECK(app_timer_create(&timeout_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout));

    app_timer_start(health_timer, APP_TIMER_TICKS(HEALTH_TIME), NULL);

    app_timer_start(timeout_timer, APP_TIMER_TICKS(1000), NULL);


    for (;;)
    {
//        nrf_delay_ms(200);
        app_sched_execute();
        (void)sd_app_evt_wait();
    }
}


