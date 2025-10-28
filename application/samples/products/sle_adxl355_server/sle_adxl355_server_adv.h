/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023. All rights reserved.
 *
 * Description: SLE advertising configuration for the ADXL355 server on BS21E.
 * This file is ported from the WS63 implementation and corrected to include
 * necessary data structure definitions.
 */

#ifndef SLE_ADXL355_SERVER_ADV_H
#define SLE_ADXL355_SERVER_ADV_H

#include "sle_device_discovery.h"
#include "errcode.h"

/*============================================================================*/
/*========================= Advertising Definitions ==========================*/
/*============================================================================*/

/**
 * @brief Advertising handle used for the server.
 */
#define SLE_ADV_HANDLE_DEFAULT               1

/**
 * @brief Structure for common advertising data elements (Type-Length-Value).
 */
struct sle_adv_common_value {
    uint8_t length;
    uint8_t type;
    uint8_t value;
};

/**
 * @brief SLE advertising channel map enumeration.
 */
typedef enum {
    SLE_ADV_CHANNEL_MAP_77           = 0x01,
    SLE_ADV_CHANNEL_MAP_78           = 0x02,
    SLE_ADV_CHANNEL_MAP_79           = 0x04,
    SLE_ADV_CHANNEL_MAP_DEFAULT      = 0x07
} sle_adv_channel_map_t;

/**
 * @brief SLE advertising data type enumeration.
 */
typedef enum {
    SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL                       = 0x01,
    SLE_ADV_DATA_TYPE_ACCESS_MODE                           = 0x02,
    SLE_ADV_DATA_TYPE_SERVICE_DATA_16BIT_UUID               = 0x03,
    SLE_ADV_DATA_TYPE_SERVICE_DATA_128BIT_UUID              = 0x04,
    SLE_ADV_DATA_TYPE_COMPLETE_LIST_OF_16BIT_SERVICE_UUIDS  = 0x05,
    SLE_ADV_DATA_TYPE_COMPLETE_LIST_OF_128BIT_SERVICE_UUIDS = 0x06,
    SLE_ADV_DATA_TYPE_INCOMPLETE_LIST_OF_16BIT_SERVICE_UUIDS= 0x07,
    SLE_ADV_DATA_TYPE_INCOMPLETE_LIST_OF_128BIT_SERVICE_UUIDS= 0x08,
    SLE_ADV_DATA_TYPE_SERVICE_STRUCTURE_HASH_VALUE          = 0x09,
    SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME                  = 0x0A,
    SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME                   = 0x0B,
    SLE_ADV_DATA_TYPE_TX_POWER_LEVEL                        = 0x0C,
    SLE_ADV_DATA_TYPE_SLB_COMMUNICATION_DOMAIN              = 0x0D,
    SLE_ADV_DATA_TYPE_SLB_MEDIA_ACCESS_LAYER_ID             = 0x0E,
    SLE_ADV_DATA_TYPE_EXTENDED                              = 0xFE,
    SLE_ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA            = 0xFF
} sle_adv_data_type;


/*============================================================================*/
/*====================== External Variable Declarations ======================*/
/*============================================================================*/

/**
 * @brief Global variable for the server's MAC address.
 * @note This must be unique for each server device. It is defined in adv.c.
 */
extern uint8_t g_sle_server_mac[SLE_ADDR_LEN];

/*============================================================================*/
/*========================= Function Prototypes ==============================*/
/*============================================================================*/

/**
 * @brief Initializes and starts the SLE advertising process.
 *
 * This function sets up the advertising parameters (intervals, power, etc.)
 * and the advertising data (discovery level, local name, etc.), then starts
 * the advertising process.
 *
 * @return ERRCODE_SLE_SUCCESS on success, or an error code on failure.
 */
errcode_t sle_adxl355_server_adv_init(void);

#endif /* SLE_ADXL355_SERVER_ADV_H */
