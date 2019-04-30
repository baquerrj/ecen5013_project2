/**
 * =================================================================================
 *    @file     common.h
 *    @brief   Defines types and functions common between the threads of the application
 *
 *
 *    @author   Roberto Baquerizo (baquerrj), roba8460@colorado.edu
 *
 *    @internal
 *       Created:  03/09/2019
 *      Revision:  none
 *      Compiler:  gcc
 *  Organization:  University of Colorado: Boulder
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 * =================================================================================
 */

#ifndef COMMON_H
#define COMMON_H


#include <stdint.h>
#include <stdlib.h>

/*******************************************************************************
 *  Controller + Remote Node Interface
 ******************************************************************************/
/*! @brief BBG and TIVA Board IDs */
#define BBG_BOARD_ID            (0x00)
#define TIVA_BOARD_ID           (0x01)

#define TIVA_SENSOR_MODULE      (1)
#define TIVA_CAMERA_MODULE      (2)
#define TIVA_COMM_MODULE        (3)
#define TIVA_LED_MODULE         (4)

#define BBG_LOGGER_MODULE       (1)
#define BBG_COMM_MODULE         (2)

/*! @brief Message ID for messages between nodes */
typedef enum
{
    NODE_MSG_ID_ALIVE = 0,
    NODE_MSG_ID_INFO,
    NODE_MSG_ID_ERROR,
    NODE_MSG_ID_WARNING,
    NODE_MSG_ID_SENSOR_STATUS,
    NODE_MSG_ID_PICTURE,
    NODE_MSG_ID_OBJECT_DETECTED,
    NODE_MSG_ID_BOARD_TYPE,
    NODE_MSG_ID_UID,

    NODE_MSG_ID_GET_SENSOR_STATUS,
    NODE_MSG_ID_GET_SENSOR_INFO,
    NODE_MSG_ID_GET_CLIENT_BOARD_TYPE,
    NODE_MSG_ID_GET_CLIENT_UID,
    NODE_MSG_ID_MAX
} node_message_e;

const char* const node_message_id_names[ NODE_MSG_ID_MAX ] =
{
    (const char*)"ALIVE",
    (const char*)"INFO",
    (const char*)"ERROR",
    (const char*)"WARNING",
    (const char*)"SENSOR_STATUS",
    (const char*)"PICTURE",
    (const char*)"OBJECT_DETECTED",
    (const char*)"BOARD_TYPE",
    (const char*)"UID",
    (const char*)"GET_SENSOR_STATUS",
    (const char*)"GET_SENSOR_INFO",
    (const char*)"GET_CLIENT_BOARD_TYPE",
    (const char*)"GET_CLIENT_UID",
};



typedef struct
{
    size_t length;
    void* frame;
} camera_packet_t;

/*! @brief Struct defining inter-node communication */
typedef struct
{
    uint8_t src_id;
    uint8_t src_brd_id;
    uint8_t dst_id;
    uint8_t dst_brd_id;
    node_message_e msg_id;
    union
    {
        float float_data;
        float sensor_value;
        camera_packet_t *camera_packet;
        size_t filler;
    } data;
    char message[18];
    uint16_t checksum;
} node_message_t;

static uint16_t getCheckSum( const node_message_t *node_msg )
{
    uint16_t checkSum = 0;
    uint8_t sizeOfPayload = sizeof( node_message_t ) - sizeof( node_msg->checksum );
    uint8_t *p_payload = (uint8_t*)node_msg;
    int i;
    for(i = 0; i < sizeOfPayload; i++)
    {
        checkSum += *( p_payload + i );
    }
    return checkSum;
}

/*!
 * @brief   Verify checksum
 *
 * @param[in]   node_msg message from remote node to verify
 * @returns 1 if a match
 */
static inline uint8_t verifyCheckSum( const node_message_t *node_msg )
{
    return getCheckSum( node_msg ) == node_msg->checksum;
}

/*!
 * @brief   Get string represenation of node message ID
 *
 * @param[in]   msg_id ID of message according to node_message_e enum
 * @return string represenation of message
 */
static inline const char* get_message_id_name( node_message_e msg_id )
{
    return node_message_id_names[ msg_id ];
}

#endif /* COMMON_H */
