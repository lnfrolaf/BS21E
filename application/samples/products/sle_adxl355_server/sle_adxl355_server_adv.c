/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023. All rights reserved.
 *
 * Description: SLE advertising configuration for the ADXL355 server on BS21E.
 * This file is ported from the WS63 implementation to ensure identical
 * advertising behavior.
 */
#include "securec.h"
#include "errcode.h"
#include "osal_addr.h"
#include "osal_debug.h"
#include "string.h"
#include "sle_common.h"
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_errcode.h"
#include "sle_adxl355_server_adv.h"

/*============================================================================*/
/*====================== Advertising Configuration ===========================*/
/*============================================================================*/

/* Connection interval 1.875ms, (15 * 125us) */
#define SLE_CONN_INTV_MIN_DEFAULT            0x0F
/* Connection interval 1.875ms, (15 * 125us) */
#define SLE_CONN_INTV_MAX_DEFAULT            0x0F
/* Advertising interval 25ms, (200 * 125us) */
#define SLE_ADV_INTERVAL_MIN_DEFAULT         0xC8
/* Advertising interval 25ms, (200 * 125us) */
#define SLE_ADV_INTERVAL_MAX_DEFAULT         0xC8
/* Supervision timeout 5000ms, unit 10ms */
#define SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT 0x1F4
/* Connection max latency */
#define SLE_CONN_MAX_LATENCY                 0x00
/* Advertising tx power */
#define SLE_ADV_TX_POWER                     10
/* Max advertising data length */
#define SLE_ADV_DATA_LEN_MAX                 251
/* sle device name */
#define NAME_MAX_LENGTH                      17 // "sle_adxl355_srv" + null

/*============================================================================*/
/*============================ Global Variables ==============================*/
/*============================================================================*/

// IMPORTANT: This MAC address must be unique for each server device.
// Ported from WS63 code.
uint8_t g_sle_server_mac[SLE_ADDR_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x06};

/* Advertising name */
static uint8_t g_sle_local_name[NAME_MAX_LENGTH] = "sle_adxl355_srv";

/*============================================================================*/
/*======================= Advertising Data Setup =============================*/
/*============================================================================*/

static uint16_t sle_set_adv_local_name(uint8_t *adv_data, uint16_t max_len)
{
    errno_t ret;
    uint8_t index = 0;

    uint8_t *local_name = g_sle_local_name;
    uint8_t local_name_len = (uint8_t)strlen((char *)local_name);

    if (local_name_len + 2 > max_len) {
        return 0;
    }

    adv_data[index++] = local_name_len + 1;
    adv_data[index++] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    ret = memcpy_s(&adv_data[index], max_len - index, local_name, local_name_len);
    if (ret != EOK) {
        osal_printk("[ADV] Set name memcpy failed\r\n");
        return 0;
    }
    return (uint16_t)index + local_name_len;
}

static uint16_t sle_set_adv_data(uint8_t *adv_data, uint16_t max_len)
{
    size_t len = 0;
    uint16_t idx = 0;
    errno_t  ret = 0;

    len = sizeof(struct sle_adv_common_value);
    if (idx + len > max_len) return idx;
    struct sle_adv_common_value adv_disc_level = {
        .length = len - 1,
        .type = SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL,
        .value = SLE_ANNOUNCE_LEVEL_NORMAL,
    };
    ret = memcpy_s(&adv_data[idx], max_len - idx, &adv_disc_level, len);
    if (ret != EOK) {
        osal_printk("[ADV] disc_level memcpy failed\r\n");
        return 0;
    }
    idx += len;

    len = sizeof(struct sle_adv_common_value);
    if (idx + len > max_len) return idx;
    struct sle_adv_common_value adv_access_mode = {
        .length = len - 1,
        .type = SLE_ADV_DATA_TYPE_ACCESS_MODE,
        .value = 0,
    };
    ret = memcpy_s(&adv_data[idx], max_len - idx, &adv_access_mode, len);
    if (ret != EOK) {
        osal_printk("[ADV] access_mode memcpy failed\r\n");
        return 0;
    }
    idx += len;

    return idx;
}

static uint16_t sle_set_scan_response_data(uint8_t *scan_rsp_data, uint16_t max_len)
{
    uint16_t idx = 0;
    errno_t ret;
    size_t scan_rsp_data_len = sizeof(struct sle_adv_common_value);

    if (idx + scan_rsp_data_len > max_len) return idx;
    struct sle_adv_common_value tx_power_level = {
        .length = scan_rsp_data_len - 1,
        .type = SLE_ADV_DATA_TYPE_TX_POWER_LEVEL,
        .value = SLE_ADV_TX_POWER,
    };
    ret = memcpy_s(scan_rsp_data, max_len, &tx_power_level, scan_rsp_data_len);
    if (ret != EOK) {
        osal_printk("[ADV] scan rsp memcpy failed\r\n");
        return 0;
    }
    idx += scan_rsp_data_len;

    /* set local name */
    idx += sle_set_adv_local_name(&scan_rsp_data[idx], max_len - idx);
    return idx;
}

static int sle_set_default_announce_param(void)
{
    sle_announce_param_t param = {0};
    param.announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE;
    param.announce_handle = SLE_ADV_HANDLE_DEFAULT;
    param.announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO;
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;
    param.announce_channel_map = SLE_ADV_CHANNEL_MAP_DEFAULT;
    param.announce_interval_min = SLE_ADV_INTERVAL_MIN_DEFAULT;
    param.announce_interval_max = SLE_ADV_INTERVAL_MAX_DEFAULT;
    param.conn_interval_min = SLE_CONN_INTV_MIN_DEFAULT;
    param.conn_interval_max = SLE_CONN_INTV_MAX_DEFAULT;
    param.conn_max_latency = SLE_CONN_MAX_LATENCY;
    param.conn_supervision_timeout = SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT;
    param.announce_tx_power = SLE_ADV_TX_POWER;
    param.own_addr.type = 0;
    memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, g_sle_server_mac, SLE_ADDR_LEN);
    return sle_set_announce_param(param.announce_handle, &param);
}

static int sle_set_default_announce_data(void)
{
    errcode_t ret;
    sle_announce_data_t data = {0};
    uint8_t adv_handle = SLE_ADV_HANDLE_DEFAULT;
    static uint8_t announce_data[SLE_ADV_DATA_LEN_MAX] = {0};
    static uint8_t seek_rsp_data[SLE_ADV_DATA_LEN_MAX] = {0};

    data.announce_data_len = sle_set_adv_data(announce_data, SLE_ADV_DATA_LEN_MAX);
    data.announce_data = announce_data;

    data.seek_rsp_data_len = sle_set_scan_response_data(seek_rsp_data, SLE_ADV_DATA_LEN_MAX);
    data.seek_rsp_data = seek_rsp_data;

    ret = sle_set_announce_data(adv_handle, &data);
    if (ret == ERRCODE_SLE_SUCCESS) {
        osal_printk("[ADV] Set announce data success.\r\n");
    } else {
        osal_printk("[ADV] Set announce data failed, ret=0x%x\r\n", ret);
    }
    return ERRCODE_SLE_SUCCESS;
}

/*============================================================================*/
/*======================= Advertising Callbacks ==============================*/
/*============================================================================*/

static void sle_announce_enable_cbk(uint32_t announce_id, errcode_t status)
{
    osal_printk("[ADV] Announce enabled: id=%02x, status=%02x\r\n", announce_id, status);
}

static void sle_announce_disable_cbk(uint32_t announce_id, errcode_t status)
{
    osal_printk("[ADV] Announce disabled: id=%02x, status=%02x\r\n", announce_id, status);
}

static void sle_announce_terminal_cbk(uint32_t announce_id)
{
    osal_printk("[ADV] Announce terminated: id=%02x\r\n", announce_id);
}

void sle_announce_register_cbks(void)
{
    sle_announce_seek_callbacks_t seek_cbks = {0};
    seek_cbks.announce_enable_cb = sle_announce_enable_cbk;
    seek_cbks.announce_disable_cb = sle_announce_disable_cbk;
    seek_cbks.announce_terminal_cb = sle_announce_terminal_cbk;
    sle_announce_seek_register_callbacks(&seek_cbks);
}

/*============================================================================*/
/*========================== Public Functions ================================*/
/*============================================================================*/

errcode_t sle_adxl355_server_adv_init(void)
{
    osal_printk("[ADV] ADV init start\r\n");
    sle_announce_register_cbks();
    sle_set_default_announce_param();
    sle_set_default_announce_data();
    sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
    osal_printk("[ADV] ADV init complete, starting announce\r\n");
    return ERRCODE_SLE_SUCCESS;
}
