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

#ifndef GENERIC_ONOFF_MESSAGES_H__
#define GENERIC_ONOFF_MESSAGES_H__

#include <stdint.h>

/**
 * @internal
 * @defgroup GENERIC_ONOFF_MESSAGES Internal header
 * @ingroup GENERIC_ONOFF_MODEL
 * This internal header contains packed structures required for message parsing.
 * @{
 */

/** Shortest allowed length for the Set message. */
#define GENERIC_ONOFF_SET_MINLEN 2
/** Longest allowed length for the Set message. */
#define GENERIC_ONOFF_SET_MAXLEN 12

/** Shortest allowed length for the Status message. */
#define GENERIC_ONOFF_STATUS_MINLEN 1
/** Longest allowed length for the Status message. */
#define GENERIC_ONOFF_STATUS_MAXLEN 7

/** Generic On Off model message opcodes. */ //GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED = 0x8203
typedef enum
{
    GENERIC_ONOFF_OPCODE_SET = 0x8202,
    GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED = 0x8203, 
    GENERIC_ONOFF_OPCODE_GET = 0x8201,
    GENERIC_ONOFF_OPCODE_STATUS = 0x8204,
    GENERIC_SENSOR_OPCODE_SET = 0x820B,
    GENERIC_SENSOR_OPCODE_STATUS = 0x820C,
    GENERIC_LIGHT_CONTROL_OPCODE = 0x8211,
    GENERIC_LIGHT_CONTROL_OPCODE_STATUS = 0x8212,
    GENERIC_OPCODE_STATUS = 0x8214,
    GENERIC_DEVICE_STATUS_OPCODE = 0x8215,
} generic_onoff_opcode_t;

/** Packed message structure typedefs are used for packing and unpacking byte stream. */

/** Message format for the generic_onoff Set message. */
typedef struct __attribute((packed))
{
    uint8_t command;
    uint8_t on_off;                                         /**< State to set */
    uint8_t tid;                                            /**< Transaction number for application */       
} generic_onoff_set_msg_pkt_t;

/** Message format for the generic_onoff Status message. */
typedef struct __attribute((packed))
{
    uint8_t type;
    uint8_t status;                                  /**< The present value of the Generic OnOff state */                                
} generic_onoff_status_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint8_t length;
    uint8_t command;
    uint8_t brightness[8];
    uint8_t tid;
} generic_light_set_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint8_t sensor_type;
    uint8_t attribute_1;
    uint8_t attribute_2;
    uint8_t attribute_3;
    uint8_t tid;
} generic_device_status_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint8_t sensor_type;
    uint8_t attribute_1;
    uint8_t attribute_2;
    uint8_t attribute_3;
    uint8_t tid;
} generic_sensor_status_msg_pkt_t;

/**@} end of GENERIC_ONOFF_MODEL_INTENRAL */
#endif /* GENERIC_ONOFF_MESSAGES_H__ */
