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

#include "generic_onoff_server.h"
#include "generic_onoff_common.h"
#include "generic_onoff_messages.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "log.h"


static uint32_t status_send(generic_onoff_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const generic_onoff_status_params_t * p_params)
{
    generic_onoff_status_msg_pkt_t msg_pkt;

   
/*
    if (p_params->present_on_off > GENERIC_ONOFF_MAX ||
        p_params->target_on_off  > GENERIC_ONOFF_MAX ||
        p_params->remaining_time_ms > TRANSITION_TIME_STEP_10M_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
*/
    msg_pkt.type = p_params->type;
    msg_pkt.status = p_params->status;

//if(p_message->opcode.opcode == GENERIC_ONOFF_OPCODE_SET)
if(p_message == NULL || p_message->opcode.opcode == GENERIC_ONOFF_OPCODE_SET)
{ //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "data %d\n", p_params->present_on_off);
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = 2,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
         if(access_reliable_model_is_free(p_server->model_handle))
         {
            return access_model_publish(p_server->model_handle, &reply);
         }
         else
         {
             return NRF_ERROR_BUSY;
         }
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}
else if(p_message->opcode.opcode == GENERIC_LIGHT_CONTROL_OPCODE)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Light %d \n", msg_pkt.status);
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_LIGHT_CONTROL_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = 2,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}
else if(p_message->opcode.opcode == GENERIC_OPCODE_STATUS)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "GENERIC_OPCODE_STATUS \n");
}
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
//    generic_onoff_server_t * p_server = (generic_onoff_server_t *)p_args;
//    generic_onoff_status_params_t out_data = {0};
//
//    p_server->settings.p_callbacks->onoff_cbs.get_cb(p_server, NULL, &out_data);
//    (void) status_send(p_server, NULL, &out_data);
}

/** Opcode Handlers */
/*
(
            (p_rx_msg->length == GENERIC_ONOFF_SET_MINLEN || p_rx_msg->length == GENERIC_ONOFF_SET_MAXLEN) &&
            (p_params->on_off <= GENERIC_ONOFF_MAX)
           );
*/
static inline bool set_params_validate(const access_message_rx_t * p_rx_msg, const generic_onoff_set_msg_pkt_t * p_params)
{
    return (
            (p_rx_msg->length >= GENERIC_ONOFF_SET_MINLEN || p_rx_msg->length <= GENERIC_ONOFF_SET_MAXLEN) &&
            (p_params->on_off <= GENERIC_ONOFF_MAX)
           );
}

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_onoff_server_t * p_server = (generic_onoff_server_t *) p_args;
    generic_onoff_set_params_t in_data = {0};
    generic_light_set_params_t light_data = {0};

    generic_onoff_status_params_t out_data = {0};

    generic_onoff_set_msg_pkt_t * p_msg_params_packed = (generic_onoff_set_msg_pkt_t *) p_rx_msg->p_data;
    generic_light_set_msg_pkt_t * p_light_msg_params_packed = (generic_light_set_msg_pkt_t *) p_rx_msg->p_data;

//   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RECEIVED\n"); 

    if(p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET || p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED)
    {
        if (set_params_validate(p_rx_msg, p_msg_params_packed))
        {
            in_data.on_off=p_msg_params_packed->on_off;
            in_data.command=p_msg_params_packed->command;
            in_data.tid = p_msg_params_packed->tid;

            if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, GENERIC_ONOFF_OPCODE_SET, in_data.tid))
            { 


                p_server->settings.p_callbacks->set_cb(p_server,
                                                            &in_data);
        
                if (p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET)
                {
                    (void) status_send(p_server, p_rx_msg, &out_data);
                }
              
            }
            else
            {
                if (p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET)
                {
                    (void) status_send(p_server, p_rx_msg, &out_data);
                }
            }
        }
    }
    else if(p_rx_msg->opcode.opcode == GENERIC_LIGHT_CONTROL_OPCODE)
    {//__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "light TID\n", p_light_msg_params_packed->tid);
        
         static uint8_t tid=0;

         if (set_params_validate(p_rx_msg, p_msg_params_packed))
         {
            light_data.length = p_light_msg_params_packed->length;
            light_data.command = p_light_msg_params_packed->command;

              memcpy(light_data.brightness, p_light_msg_params_packed->brightness, p_light_msg_params_packed->length-3);
            light_data.tid = p_light_msg_params_packed->tid;
            if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, GENERIC_LIGHT_CONTROL_OPCODE, light_data.tid))
            {  
                 p_server->settings.p_callbacks->light_set_cb(p_server,
                                                            &light_data);

                if (p_rx_msg->opcode.opcode == GENERIC_LIGHT_CONTROL_OPCODE)
                {
                   
                   (void) status_send(p_server, p_rx_msg, &out_data);
                }

               
            }
            else
            {
                if (p_rx_msg->opcode.opcode == GENERIC_LIGHT_CONTROL_OPCODE)
                {
                    (void) status_send(p_server, p_rx_msg, &out_data);
                }
            }
         }
    }
}


static inline bool get_params_validate(const access_message_rx_t * p_rx_msg)
{
    return (p_rx_msg->length == 0);
}

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_onoff_server_t * p_server = (generic_onoff_server_t *) p_args;
    generic_onoff_status_params_t out_data = {0};

    if (get_params_validate(p_rx_msg))
    {
        p_server->settings.p_callbacks->get_cb(p_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_LIGHT_CONTROL_OPCODE), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(GENERIC_OPCODE_STATUS), status_send},
};


/** Interface functions */
uint32_t generic_onoff_server_init(generic_onoff_server_t * p_server, uint8_t element_index)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->set_cb == NULL ||
        p_server->settings.p_callbacks->get_cb == NULL )
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(GENERIC_ONOFF_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    uint32_t status = access_model_add(&init_params, &p_server->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    return status;
}

uint32_t generic_onoff_server_status_publish(generic_onoff_server_t * p_server, const generic_onoff_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}


static uint8_t message_set_packet_create(generic_device_status_msg_pkt_t *p_set, const generic_sensor_status_params_t * p_params)
{
       p_set->sensor_type = p_params->sensor_type;
       p_set->attribute_1 = p_params->attribute_1;
       p_set->attribute_2 = p_params->attribute_2;
       p_set->attribute_3 = p_params->attribute_3;
       p_set->tid = p_params->tid;

        return 5;
}


static void message_create(generic_onoff_server_t * p_server, uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t *p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_server->settings.force_segmented;
    p_message->transmic_size = p_server->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(generic_onoff_server_t * p_server, uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_server->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    p_reliable->status_cb = p_server->settings.p_callbacks->ack_transaction_status_cb;
}

uint32_t generic_sensor_status_publish(generic_onoff_server_t * p_server, const generic_sensor_status_params_t * p_params)
{
    //generic_sensor_status_msg_pkt_t msg_pkt;

    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if(access_reliable_model_is_free(p_server->model_handle))
    {

            uint8_t length = message_set_packet_create(&p_server->sensor_pkt.set, p_params);

            message_create(p_server, GENERIC_SENSOR_OPCODE_STATUS, (const uint8_t *) &p_server->sensor_pkt.set,
                       length, &p_server->access_message.message);

            reliable_context_create(p_server, GENERIC_OPCODE_STATUS, &p_server->access_message);

        return access_model_reliable_publish(&p_server->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}


uint32_t generic_device_status_publish(generic_onoff_server_t * p_server, const generic_sensor_status_params_t * p_params)
{
    //generic_device_status_msg_pkt_t msg_pkt;

    if (p_server == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if(access_reliable_model_is_free(p_server->model_handle))
    {

            uint8_t length = message_set_packet_create(&p_server->device_pkt.set, p_params);

            message_create(p_server, GENERIC_DEVICE_STATUS_OPCODE, (const uint8_t *) &p_server->device_pkt.set,
                       length, &p_server->access_message.message);

            reliable_context_create(p_server, GENERIC_OPCODE_STATUS, &p_server->access_message);

        return access_model_reliable_publish(&p_server->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}