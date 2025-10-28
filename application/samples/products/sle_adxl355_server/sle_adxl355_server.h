/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023. All rights reserved.
 *
 * Description: Header file for the SLE ADXL355 Server on BS21E.
 * This file is ported from the WS63 implementation.
 */

#ifndef SLE_ADXL355_SERVER_H
#define SLE_ADXL355_SERVER_H

#include "sle_ssap_server.h"
#include "errcode.h"

/*============================================================================*/
/*========================= Service & Property UUIDs =========================*/
/*============================================================================*/

/* Service UUID */
#define SLE_UUID_SERVER_SERVICE        0xABCD

/* Property UUIDs */
#define SLE_UUID_SERVER_NTF_REPORT     0x1122
#define SLE_UUID_SERVER_CMD_CONTROL    0x1123
#define SLE_UUID_SERVER_STATUS_REPORT  0x1124

/*============================================================================*/
/*========================= Function Prototypes ==============================*/
/*============================================================================*/

/**
 * @brief Initializes the entire ADXL355 SLE server application.
 *
 * This function sets up SLE callbacks, services, properties, hardware (I2C, GPIO),
 * and creates the necessary RTOS tasks for sensor sampling and data transmission.
 * It is the main entry point for the server logic after the SLE stack is enabled.
 *
 * @return ERRCODE_SLE_SUCCESS on success, or an error code on failure.
 */
errcode_t sle_adxl355_server_init(void);

/**
 * @brief Sends a data report (notification) to the connected client using a handle.
 *
 * @param data Pointer to the data buffer to be sent.
 * @param len Length of the data in bytes.
 * @param connect_id The connection ID of the target client.
 * @return ERRCODE_SLE_SUCCESS on success, or an error code on failure.
 */
errcode_t sle_uuid_server_send_report_by_handle_id(uint8_t *data, uint16_t len, uint16_t connect_id);

#endif /* SLE_ADXL355_SERVER_H */
