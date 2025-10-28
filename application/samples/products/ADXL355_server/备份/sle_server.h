/*
 * Copyright (c) @CompanyNameMagicTag 2023-2023. All rights reserved.
 *
 * Description: Merged and Refactored header for ADXL355 SLE Server.
 * Author: @CompanyNameTag
 * History:
 * 2023-07-24, Create file by merging and refactoring existing sources.
 */

#ifndef SLE_SERVER_H
#define SLE_SERVER_H

#include <stdint.h>
#include "errcode.h"
#include "sle_ssap_server.h"
#include "pinctrl.h"
#include "gpio.h"
#include "soc_osal.h" /* IMPORTANT: Include for OSAL types */

/* ============================================================================ */
/* ==================== MERGED DEFINITIONS FROM ALL FILES ===================== */
/* ============================================================================ */

/* ---------------- From sle_uart_server_adv.h ---------------- */
typedef enum sle_adv_channel {
    SLE_ADV_CHANNEL_MAP_77                 = 0x01,
    SLE_ADV_CHANNEL_MAP_78                 = 0x02,
    SLE_ADV_CHANNEL_MAP_79                 = 0x04,
    SLE_ADV_CHANNEL_MAP_DEFAULT            = 0x07
} sle_adv_channel_map_t;

typedef enum sle_adv_data {
    SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL                              = 0x01,
    SLE_ADV_DATA_TYPE_ACCESS_MODE                                  = 0x02,
    SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME                          = 0x0B,
    SLE_ADV_DATA_TYPE_TX_POWER_LEVEL                               = 0x0C,
    SLE_ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA                   = 0xFF
} sle_adv_data_type;

/* ---------------- From ADXL355_I2C.h ---------------- */
// DRDY（Data Ready）Pin Configuration
#define ADXL355_DRDY_PIN           CONFIG_ADXL355_DRDY_PIN
#define ADXL355_DRDY_FUNC          HAL_PIO_FUNC_GPIO

// Register lengths
typedef enum {
    ADXL355_REGS_1 = 1,
    ADXL355_REGS_2 = 2,
    ADXL355_REGS_3 = 3
} adxl355_reglen_t;

/* ---------------- From sle_server.h ---------------- */
/**
 * @brief This is the callback function when the ADXL355 DRDY (Data Ready)
 * pin triggers an interrupt, indicating that new sensor data is available.
 * @param pin The GPIO pin number that triggered the interrupt.
 * @param param User-defined parameter (unused).
 */
void adxl355_drdy_cb(pin_t pin, uintptr_t param);

#endif // SLE_SERVER_H
