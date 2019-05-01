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

#include <stdlib.h>

/*******************************************************************************
 *  Controller + Remote Node Interface
 ******************************************************************************/
/*! @brief BBG and TIVA Board IDs */


typedef enum
{
    BOARD_ID_BBG    = 0,
    BOARD_ID_TIVA   = 1,
    BOARD_ID_MAX
} board_e;


typedef enum
{
    TIVA_MODULE_TMP102   = 0,
    TIVA_MODULE_APDS9301,
    TIVA_MODULE_CAMERA,
    TIVA_MODULE_COMM,
    TIVA_MODULE_LED,
    TIVA_MODULE_MAX
} tiva_module_e;


typedef enum
{
    BBG_MODULE_LOGGER = 0,
    BBG_MODULE_COMM,
    BBG_MODULE_MAX
} bbg_module_e;


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

    NODE_MSG_ID_GET_TEMPERATURE,
    NODE_MSG_ID_GET_LUX,
    NODE_MSG_ID_GET_SENSOR_INFO,
    NODE_MSG_ID_GET_CLIENT_BOARD_TYPE,
    NODE_MSG_ID_GET_CLIENT_UID,
    NODE_MSG_ID_MAX
} node_message_e;


#define BBG_BOARD_ID        BOARD_ID_BBG
#define TIVA_BOARD_ID       BOARD_ID_TIVA

#define TIVA_TMP102_MODULE      TIVA_MODULE_TMP102
#define TIVA_APDS9301_MODULE    TIVA_MODULE_APDS9301
#define TIVA_CAMERA_MODULE      TIVA_MODULE_CAMERA
#define TIVA_COMM_MODULE        TIVA_MODULE_COMM
#define TIVA_LED_MODULE         TIVA_MODULE_LED

#define BBG_LOGGER_MODULE       BBG_MODULE_LOGGER

static const char* const node_message_id_names[ NODE_MSG_ID_MAX ] =
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
    (const char*)"GET_TEMPERATURE",
    (const char*)"GET_LUX",
    (const char*)"GET_SENSOR_INFO",
    (const char*)"GET_CLIENT_BOARD_TYPE",
    (const char*)"GET_CLIENT_UID",
};

static const char* const board_id_names[ BOARD_ID_MAX ] =
{
    (const char*)"BOARD_ID_BBG",
    (const char*)"BOARD_ID_TIVA",
};

static const char* const tiva_module_names[ TIVA_MODULE_MAX ] =
{
    (const char*)"TIVA_MODULE_TMP102",
    (const char*)"TIVA_MODULE_APDS9301",
    (const char*)"TIVA_MODULE_CAMERA",
    (const char*)"TIVA_MOUDLE_COMM",
    (const char*)"TIVA_MODULE_LED",
};

static const char* const bbg_module_names[ BBG_MODULE_MAX ] =
{
    (const char*)"BBG_MODULE_LOGGER",
    (const char*)"BBG_MODULE_COMM",
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

static inline const char* get_board_id_name( board_e board_id )
{
    return board_id_names[ board_id ];
}

static inline const char* get_tiva_module_name( tiva_module_e module_id )
{
    return tiva_module_names[ module_id ];
}

static inline const char* get_bbg_module_name( bbg_module_e module_id )
{
    return bbg_module_names[ module_id ];
}



#endif /* COMMON_H */
