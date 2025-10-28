/**
 * @file server.c
 * @brief SLE server for ADXL355 accelerometer data transmission via I2C on BS21E.
 * @version 1.7
 *
 * ===================================================================
 * V1.7 Change Log (Compilation and Startup Fix):
 * - Fixed compilation errors by using the correct API for SLE stack
 * initialization: `sle_dev_manager_register_callbacks` and the
 * `sle_dev_manager_callbacks_t` type.
 * - Reinstated the full, robust startup sequence where the application
 * entry point registers power-on and enable callbacks.
 * - The `sle_power_on_cbk` now correctly calls `enable_sle()`, which
 * was a critical missing step causing the previous hang.
 * - The `sle_enable_cbk` releases the semaphore, ensuring the main
 * application task only proceeds after the stack is confirmed to be ready.
 * - This version provides a stable, race-condition-free, and
 * compilable initialization flow.
 * ===================================================================
 */

/*============================================================================*/
/*============================= C Standard Includes ==========================*/
/*============================================================================*/
#include "string.h"
#include <stdio.h>

/*============================================================================*/
/*============================= SLE/OS Includes ==============================*/
/*============================================================================*/
#include "app_init.h"
#include "securec.h"
#include "errcode.h"
#include "common_def.h"
#include "soc_osal.h"
#include "osal_addr.h"
#include "osal_debug.h"
#include "osal_task.h"

// Hardware and RTOS headers
#include "pinctrl.h"
#include "gpio.h"
#include "i2c.h"
#include "tcxo.h"
#include "timer.h"
#include "cmsis_os2.h"
#include "nv.h"

// SLE headers
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_ssap_server.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_device_manager.h"

// Application-specific headers
#include "sle_adxl355_server.h"
#include "sle_adxl355_server_adv.h"

/*============================================================================*/
/*========================= ADXL355 Driver Defines ===========================*/
/*============================================================================*/
/* ADXL355 registers addresses */
#define ADXL355_DEVID_AD                0x00
#define ADXL355_DEVID_MST               0x01
#define ADXL355_PARTID                  0x02
#define ADXL355_REVID                   0x03
#define ADXL355_STATUS                  0x04
#define ADXL355_FIFO_ENTRIES            0x05
#define ADXL355_TEMP2                   0x06
#define ADXL355_TEMP1                   0x07
#define ADXL355_XDATA3                  0x08
#define ADXL355_XDATA2                  0x09
#define ADXL355_XDATA1                  0x0A
#define ADXL355_YDATA3                  0x0B
#define ADXL355_YDATA2                  0x0C
#define ADXL355_YDATA1                  0x0D
#define ADXL355_ZDATA3                  0x0E
#define ADXL355_ZDATA2                  0x0F
#define ADXL355_ZDATA1                  0x10
#define ADXL355_FIFO_DATA               0x11
#define ADXL355_OFFSET_X_H              0x1E
#define ADXL355_OFFSET_X_L              0x1F
#define ADXL355_OFFSET_Y_H              0x20
#define ADXL355_OFFSET_Y_L              0x21
#define ADXL355_OFFSET_Z_H              0x22
#define ADXL355_OFFSET_Z_L              0x23
#define ADXL355_ACT_EN                  0x24
#define ADXL355_ACT_THRESH_H            0x25
#define ADXL355_ACT_THRESH_L            0x26
#define ADXL355_ACT_COUNT               0x27
#define ADXL355_FILTER                  0x28
#define ADXL355_FIFO_SAMPLES            0x29
#define ADXL355_INT_MAP                 0x2A
#define ADXL355_SYNC                    0x2B
#define ADXL355_RANGE                   0x2C
#define ADXL355_POWER_CTL               0x2D
#define ADXL355_SELF_TEST               0x2E
#define ADXL355_RESET                   0x2F

/* ODR values */
#define ADXL355_ODR_62_5                ((uint16_t)0x06)

/* Range values */
#define ADXL355_RANGE_2G                ((uint16_t)0x81)

typedef enum {
    ADXL355_REGS_1 = 1,
    ADXL355_REGS_2 = 2,
    ADXL355_REGS_3 = 3
} adxl355_reglen_t;

typedef struct {
    uint8_t ADXL355_Range;
    uint16_t ADXL355_LowPass;
    uint16_t ADXL355_HighPass;
} ADXL355_HandleTypeDef;

/*============================================================================*/
/*========================= Application Configuration ========================*/
/*============================================================================*/
#define UUID_LEN_2 2

#define encode2byte_little(_ptr, data) \
    do { \
        *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 8); \
        *(uint8_t *)(_ptr) = (uint8_t)(data); \
    } while (0)

/* Task & System Configuration */
#define MAIN_TASK_STACK_SIZE             0x2000
#define DATA_SENDING_TASK_STACK_SIZE     0x2000
#define SENSOR_SAMPLING_TASK_STACK_SIZE  0x1000
#define SENSOR_SAMPLING_TASK_PRIO        25
#define DATA_SENDING_TASK_PRIO           25
#define MAIN_TASK_PROI                   26
#define INITIAL_COLLECTION_START_COUNT   10
#define INITIAL_TRANSMISSION_START_COUNT 10

/* Hardware & Data Configuration */
#define SENSOR_DATA_BUFFER_SIZE          4096
#define DEFAULT_SLE_SPEED_MTU_SIZE       280
#define PACKET_SEND_INTERVAL_MS          10

/* Custom Framing Definitions */
#define MTU_PACKET_END_FLAG              0xB4
#define TRANSMISSION_END_FLAG            0x4B

#define MTU_HEADER_SIZE                  8
#define MTU_TRAILER_SIZE                 1
#define ATT_HEADER_OVERHEAD              3
#define MTU_OVERHEAD_SIZE                (MTU_HEADER_SIZE + MTU_TRAILER_SIZE)
#define MTU_PAYLOAD_SIZE                 (DEFAULT_SLE_SPEED_MTU_SIZE - MTU_OVERHEAD_SIZE - ATT_HEADER_OVERHEAD)

/* ADXL355 PIN Definitions (Must be configured in board config) */
#define ADXL355_DRDY_PIN                 CONFIG_ADXL355_DRDY_PIN
#define ADXL355_DRDY_FUNC                0

// I2C configuration defines
#define ADXL355_I2C_BUS_ID               CONFIG_I2C_MASTER_BUS_ID
#define ADXL355_I2C_BAUDRATE             400000
#define ADXL355_I2C_ADDR                 0x1D

/*============================================================================*/
/*============================ Global Variables ==============================*/
/*============================================================================*/
static bool g_is_sle_connected = false;
static osSemaphoreId_t g_send_data_sem = NULL;
static osSemaphoreId_t g_drdy_sem = NULL;
static osSemaphoreId_t g_sle_enabled_sem = NULL;
static volatile bool g_is_sending_data = false;
static uint16_t g_sle_conn_hdl = 0;
static uint8_t g_server_id = 0;
static uint16_t g_service_handle = 0;
static uint16_t g_property_handle = 0;
static uint16_t g_command_property_handle = 0;
static uint16_t g_status_property_handle = 0;
static uint8_t g_device_unique_id = 0;
static char g_sle_uuid_app_uuid[UUID_LEN_2] = {0xAB, 0xCD};
static uint8_t sle_uuid_base[] = { 0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA, \
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint32_t g_start_command_count = 0;
static volatile bool g_collection_phase_started = false;
static volatile bool g_is_operational = false;
int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
ADXL355_HandleTypeDef ADXL355_t;
#define SENSOR_SAMPLE_SIZE_BYTES (3 * sizeof(int32_t))
#define SENSOR_BUFFER_SAMPLES (SENSOR_DATA_BUFFER_SIZE / SENSOR_SAMPLE_SIZE_BYTES)
static int32_t g_sensor_data_buffer[SENSOR_BUFFER_SAMPLES * 3];
static volatile uint32_t g_sensor_buffer_head = 0;
static volatile uint32_t g_sensor_buffer_tail = 0;

/*============================================================================*/
/*======================= ADXL355 Driver Implementation ======================*/
/*============================================================================*/
int32_t ADXL355_AccDataConversion(uint32_t ui32SensorData) {
    int32_t volatile i32Conversion = 0;
    ui32SensorData = (ui32SensorData >> 4);
    ui32SensorData = (ui32SensorData & 0x000FFFFF);
    if ((ui32SensorData & 0x00080000) == 0x00080000) {
        i32Conversion = (ui32SensorData | 0xFFF00000);
    } else {
        i32Conversion = ui32SensorData;
    }
    return i32Conversion;
}

void ADXL355_WriteRegister(uint8_t reg, uint8_t val) {
    i2c_data_t data = {0};
    uint8_t tx[2] = {reg, val};
    data.send_buf = tx;
    data.send_len = 2;
    data.receive_buf = NULL;
    data.receive_len = 0;
    uapi_i2c_master_write(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
}

uint32_t ADXL355_ReadRegister(uint8_t reg, adxl355_reglen_t len) {
    i2c_data_t data = {0};
    uint8_t tx_addr = reg;
    uint8_t rx_buf[3] = {0};
    uint32_t result = 0;
    data.send_buf = &tx_addr;
    data.send_len = 1;
    data.receive_buf = rx_buf;
    data.receive_len = len;
    uapi_i2c_master_write(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
    uapi_i2c_master_read(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
    for (uint8_t i = 0; i < len; i++) {
        result = (result << 8) | rx_buf[i];
    }
    return result;
}

static void ADXL355_Standby(void) {
    uint8_t ui8temp;
    ui8temp = (uint8_t)ADXL355_ReadRegister(ADXL355_POWER_CTL, ADXL355_REGS_1);
    ui8temp |= 0x01;
    ADXL355_WriteRegister(ADXL355_POWER_CTL, ui8temp);
}

static void ADXL355_Startup(void) {
    uint8_t ui8temp;
    ui8temp = (uint8_t)ADXL355_ReadRegister(ADXL355_POWER_CTL, ADXL355_REGS_1);
    ui8temp &= ~(0x01);
    ADXL355_WriteRegister(ADXL355_POWER_CTL, ui8temp);
}

void ADXL355_ReadData(void) {
    uint8_t buf[9] = {0};
    i2c_data_t data = {0};
    uint8_t reg = ADXL355_XDATA3;
    data.send_buf = &reg;
    data.send_len = 1;
    data.receive_buf = buf;
    data.receive_len = 9;
    uapi_i2c_master_write(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
    uapi_i2c_master_read(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
    ui32SensorX = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    ui32SensorY = (buf[3] << 16) | (buf[4] << 8) | buf[5];
    ui32SensorZ = (buf[6] << 16) | (buf[7] << 8) | buf[8];
    i32SensorX = ADXL355_AccDataConversion(ui32SensorX);
    i32SensorY = ADXL355_AccDataConversion(ui32SensorY);
    i32SensorZ = ADXL355_AccDataConversion(ui32SensorZ);
}

void ADXL355_Init(void) {
    ADXL355_t.ADXL355_Range = ADXL355_RANGE_2G;
    ADXL355_t.ADXL355_LowPass = ADXL355_ODR_62_5;
    ADXL355_t.ADXL355_HighPass = 0x00;
    ADXL355_Standby();
    ADXL355_WriteRegister(ADXL355_RANGE, ADXL355_t.ADXL355_Range);
    uint8_t filter_val = (ADXL355_t.ADXL355_HighPass << 4) | ADXL355_t.ADXL355_LowPass;
    ADXL355_WriteRegister(ADXL355_FILTER, filter_val);
    ADXL355_Startup();
    osal_printk("[ADXL355] Initialized and started.\r\n");
}

/*============================================================================*/
/*==================== Application Core Logic Implementation =================*/
/*============================================================================*/
void adxl355_drdy_cb(pin_t pin, uintptr_t param) {
    unused(pin);
    unused(param);
    if (g_collection_phase_started) {
        osSemaphoreRelease(g_drdy_sem);
    }
}

void ADXL355_DRDY_Init(void) {
    uapi_pin_set_mode(ADXL355_DRDY_PIN, ADXL355_DRDY_FUNC);
    uapi_gpio_set_dir(ADXL355_DRDY_PIN, GPIO_DIRECTION_INPUT);
    uapi_gpio_register_isr_func(ADXL355_DRDY_PIN, GPIO_INTERRUPT_RISING_EDGE, adxl355_drdy_cb);
    uapi_gpio_enable_interrupt(ADXL355_DRDY_PIN);
    osal_printk("[INIT] ADXL355 DRDY interrupt initialized.\r\n");
}

static void ADXL355_HardwareInit(void) {
    uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, HAL_PIO_I2C0_CLK);
    uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, HAL_PIO_I2C0_DATA);
    uapi_pin_set_pull(CONFIG_I2C_SCL_MASTER_PIN, PIN_PULL_UP);
    uapi_pin_set_pull(CONFIG_I2C_SDA_MASTER_PIN, PIN_PULL_UP);
    uapi_tcxo_delay_ms(10);
    uapi_i2c_master_init(ADXL355_I2C_BUS_ID, ADXL355_I2C_BAUDRATE, 0);
    osal_printk("[INIT] ADXL355 I2C hardware initialized.\r\n");
}

static void *sensor_sampling_task(const char *arg) {
    unused(arg);
    static bool is_sensor_initialized = false;
    osal_printk("[TASK] Sensor Sampling Task started.\r\n");
    while (1) {
        if (!g_collection_phase_started) {
            osal_msleep(500);
            continue;
        }
        if (!is_sensor_initialized) {
            ADXL355_Init();
            is_sensor_initialized = true;
            osal_printk("[TASK] Collection started. ADXL355 Initialized.\r\n");
        }
        osSemaphoreAcquire(g_drdy_sem, osWaitForever);
        if (!g_collection_phase_started) {
            is_sensor_initialized = false;
            continue;
        }
        ADXL355_ReadData();
        uint32_t head = g_sensor_buffer_head;
        uint32_t tail = g_sensor_buffer_tail;
        uint32_t next_tail = (tail + 1) % SENSOR_BUFFER_SAMPLES;
        if (next_tail != head) {
            uint32_t index = tail * 3;
            g_sensor_data_buffer[index] = i32SensorX;
            g_sensor_data_buffer[index + 1] = i32SensorY;
            g_sensor_data_buffer[index + 2] = i32SensorZ;
            g_sensor_buffer_tail = next_tail;
        }
    }
    return NULL;
}

void dequeue_and_send_sensor_data(void) {
    if (!g_is_sle_connected || g_is_sending_data) return;
    g_is_sending_data = true;
    static uint8_t tx_linear_buffer[SENSOR_DATA_BUFFER_SIZE];
    uint32_t total_bytes_to_send = 0;
    unsigned int irq_status = osal_irq_lock();
    uint32_t head = g_sensor_buffer_head;
    uint32_t tail = g_sensor_buffer_tail;
    if (head != tail) {
        uint32_t samples_available;
        if (tail > head) {
            samples_available = tail - head;
        } else {
            samples_available = SENSOR_BUFFER_SAMPLES - head + tail;
        }
        total_bytes_to_send = samples_available * SENSOR_SAMPLE_SIZE_BYTES;
        if (total_bytes_to_send > 0 && total_bytes_to_send <= SENSOR_DATA_BUFFER_SIZE) {
            uint32_t samples_to_copy = total_bytes_to_send / SENSOR_SAMPLE_SIZE_BYTES;
            if (tail > head || (head + samples_to_copy) <= SENSOR_BUFFER_SAMPLES) {
                memcpy_s(tx_linear_buffer, sizeof(tx_linear_buffer), &g_sensor_data_buffer[head * 3], total_bytes_to_send);
            } else {
                uint32_t part1_samples = SENSOR_BUFFER_SAMPLES - head;
                uint32_t part1_bytes = part1_samples * SENSOR_SAMPLE_SIZE_BYTES;
                memcpy_s(tx_linear_buffer, sizeof(tx_linear_buffer), &g_sensor_data_buffer[head * 3], part1_bytes);
                uint32_t part2_bytes = total_bytes_to_send - part1_bytes;
                if (part2_bytes > 0) {
                    memcpy_s(&tx_linear_buffer[part1_bytes], sizeof(tx_linear_buffer) - part1_bytes, g_sensor_data_buffer, part2_bytes);
                }
            }
        } else {
            total_bytes_to_send = 0;
        }
        g_sensor_buffer_head = tail;
    }
    osal_irq_restore(irq_status);
    if (total_bytes_to_send > 0) {
        osal_printk("[TX] Dequeued %u aligned bytes.\r\n", total_bytes_to_send);
        uint64_t start_time_tx = uapi_tcxo_get_ms();
        uint8_t packet_buffer[DEFAULT_SLE_SPEED_MTU_SIZE];
        uint16_t total_packets = (total_bytes_to_send + MTU_PAYLOAD_SIZE - 1) / MTU_PAYLOAD_SIZE;
        if (total_packets == 0 && total_bytes_to_send > 0) total_packets = 1;
        uint32_t sent_bytes = 0;
        uint16_t packet_count = 0;
        while (sent_bytes < total_bytes_to_send) {
            packet_count++;
            uint16_t bytes_for_this_payload = MTU_PAYLOAD_SIZE;
            if (sent_bytes + bytes_for_this_payload > total_bytes_to_send) {
                bytes_for_this_payload = total_bytes_to_send - sent_bytes;
            }
            uint16_t total_packet_size = MTU_HEADER_SIZE + bytes_for_this_payload + MTU_TRAILER_SIZE;
            uint8_t* p = packet_buffer;
            *p++ = (uint8_t)(total_packets >> 8); *p++ = (uint8_t)(total_packets & 0xFF);
            *p++ = (uint8_t)(packet_count >> 8); *p++ = (uint8_t)(packet_count & 0xFF);
            uint32_t timestamp = (uint32_t)uapi_tcxo_get_ms();
            *p++ = (uint8_t)(timestamp >> 24); *p++ = (uint8_t)(timestamp >> 16);
            *p++ = (uint8_t)(timestamp >> 8); *p++ = (uint8_t)(timestamp & 0xFF);
            memcpy_s(p, sizeof(packet_buffer) - MTU_HEADER_SIZE, &tx_linear_buffer[sent_bytes], bytes_for_this_payload);
            p += bytes_for_this_payload;
            *p = (packet_count == total_packets) ? TRANSMISSION_END_FLAG : MTU_PACKET_END_FLAG;
            errcode_t ret = sle_uuid_server_send_report_by_handle_id(packet_buffer, total_packet_size, g_sle_conn_hdl);
            if (ret != ERRCODE_SUCC) {
                osal_printk("[TX] Failed to send packet %u. Error: %d\r\n", packet_count, ret);
                break;
            }
            sent_bytes += bytes_for_this_payload;
            osal_msleep(PACKET_SEND_INTERVAL_MS);
        }
        uint64_t end_time_tx = uapi_tcxo_get_ms();
        osal_printk("[TX] Finished. Pkts: %u. Time: %u ms\r\n", packet_count, (uint32_t)(end_time_tx - start_time_tx));
    } else {
        osal_printk("[TX] No new data. Sending empty completion packet.\r\n");
        uint8_t empty_packet[MTU_OVERHEAD_SIZE];
        uint8_t* p = empty_packet;
        *p++ = 0; *p++ = 1;
        *p++ = 0; *p++ = 1;
        uint32_t timestamp = (uint32_t)uapi_tcxo_get_ms();
        *p++ = (uint8_t)(timestamp >> 24); *p++ = (uint8_t)(timestamp >> 16);
        *p++ = (uint8_t)(timestamp >> 8); *p++ = (uint8_t)(timestamp & 0xFF);
        *p = TRANSMISSION_END_FLAG;
        sle_uuid_server_send_report_by_handle_id(empty_packet, sizeof(empty_packet), g_sle_conn_hdl);
    }
    g_is_sending_data = false;
}

static void *data_sending_task(const char *arg) {
    unused(arg);
    osal_printk("[TASK] Data Sending Task started.\r\n");
    while (1) {
        osSemaphoreAcquire(g_send_data_sem, osWaitForever);
        dequeue_and_send_sensor_data();
    }
    return NULL;
}

/*============================================================================*/
/*======================== SLE Callbacks & Functions =========================*/
/*============================================================================*/
static void ssaps_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para, errcode_t status) {
    unused(server_id);
    unused(conn_id);
    if (status != ERRCODE_SUCC || write_cb_para == NULL || write_cb_para->value == NULL) return;
    if (write_cb_para->handle == g_command_property_handle) {
        uint8_t command = write_cb_para->value[0];
        if (command == 0x01) {
            if (g_is_operational) {
                osal_printk("[SERVER] Received START. Triggering data send.\r\n");
                osSemaphoreRelease(g_send_data_sem);
                return;
            }
            g_start_command_count++;
            osal_printk("[SERVER] Received initial START (%u/%u).\r\n", g_start_command_count, INITIAL_TRANSMISSION_START_COUNT);
            if (!g_collection_phase_started && g_start_command_count >= INITIAL_COLLECTION_START_COUNT) {
                osal_printk("[SERVER] Collection threshold reached. Starting sampling.\r\n");
                g_collection_phase_started = true;
            }
            if (!g_is_operational && g_start_command_count >= INITIAL_TRANSMISSION_START_COUNT) {
                osal_printk("[SERVER] Transmission threshold reached. System operational.\r\n");
                g_is_operational = true;
                osSemaphoreRelease(g_send_data_sem);
            }
        }
    }
}

static void sle_uuid_set_base(sle_uuid_t *out) {
    (void)memcpy_s(out->uuid, SLE_UUID_LEN, sle_uuid_base, SLE_UUID_LEN);
    out->len = UUID_LEN_2;
}

static void sle_uuid_setu2(uint16_t u2, sle_uuid_t *out) {
    sle_uuid_set_base(out);
    out->len = UUID_LEN_2;
    encode2byte_little(&out->uuid[14], u2);
}

static void ssaps_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para, errcode_t status) {
    osal_printk("[SERVER] Read req: sid=%x, cid=%x, hdl=%x, stat=%x\r\n", server_id, conn_id, read_cb_para->handle, status);
}

static void ssaps_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id, ssap_exchange_info_t *mtu_size, errcode_t status) {
    osal_printk("[SERVER] MTU changed: sid=%d, cid=%d, mtu=%d, stat=%d\r\n", server_id, conn_id, mtu_size->mtu_size, status);
}

static void ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status) {
    osal_printk("[SERVER] Service started: sid=%d, hdl=%d, stat=%d\r\n", server_id, handle, status);
}

static void sle_ssaps_register_cbks(void) {
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.start_service_cb = ssaps_start_service_cbk;
    ssaps_cbk.mtu_changed_cb = ssaps_mtu_changed_cbk;
    ssaps_cbk.read_request_cb = ssaps_read_request_cbk;
    ssaps_cbk.write_request_cb = ssaps_write_request_cbk;
    ssaps_register_callbacks(&ssaps_cbk);
}

errcode_t sle_uuid_server_send_report_by_handle_id(uint8_t *data, uint16_t len, uint16_t connect_id) {
    ssaps_ntf_ind_t param = {0};
    param.handle = g_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = data;
    param.value_len = len;
    return ssaps_notify_indicate(g_server_id, connect_id, &param);
}

static errcode_t sle_uuid_server_service_add(void) {
    sle_uuid_t service_uuid = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);
    errcode_t ret = ssaps_add_service_sync(g_server_id, &service_uuid, 1, &g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("[SERVER] Add service failed: %x\r\n", ret);
    }
    return ret;
}

static errcode_t sle_uuid_server_property_add(void) {
    errcode_t ret;
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};
    uint8_t ntf_value[] = {0x01, 0x0};
    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &property.uuid);
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    descriptor.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    descriptor.value = ntf_value;
    descriptor.value_len = sizeof(ntf_value);
    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    memset_s(&property, sizeof(ssaps_property_info_t), 0, sizeof(ssaps_property_info_t));
    property.permissions = SSAP_PERMISSION_WRITE | SSAP_PERMISSION_ENCRYPTION_NEED;
    sle_uuid_setu2(SLE_UUID_SERVER_CMD_CONTROL, &property.uuid);
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_command_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    memset_s(&property, sizeof(ssaps_property_info_t), 0, sizeof(ssaps_property_info_t));
    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    sle_uuid_setu2(SLE_UUID_SERVER_STATUS_REPORT, &property.uuid);
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_status_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_status_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t sle_uuid_server_add(void) {
    sle_uuid_t app_uuid = {0};
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    ssaps_register_server(&app_uuid, &g_server_id);
    if (sle_uuid_server_service_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    if (sle_uuid_server_property_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    errcode_t ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("[SERVER] Start service failed: %x\r\n", ret);
        return ERRCODE_SLE_FAIL;
    }
    return ERRCODE_SLE_SUCCESS;
}

static void sle_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr, sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason) {
    unused(addr);
    osal_printk("[SERVER] Conn state: cid=0x%02x, state=0x%x, pair=0x%x, reason=0x%x\r\n", conn_id, conn_state, pair_state, disc_reason);
    g_sle_conn_hdl = conn_id;
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        g_is_sle_connected = true;
        g_collection_phase_started = false;
        g_is_operational = false;
        g_start_command_count = 0;
        osal_printk("[SERVER] Device connected. Waiting for initial commands.\r\n");
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        g_is_sle_connected = false;
        g_collection_phase_started = false;
        g_is_operational = false;
        unsigned int irq_status = osal_irq_lock();
        g_sensor_buffer_head = 0;
        g_sensor_buffer_tail = 0;
        osal_irq_restore(irq_status);
        osal_printk("[SERVER] Device disconnected. Buffers and state reset.\r\n");
        sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
    }
}

static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status) {
    unused(addr);
    osal_printk("[SERVER] Pair complete: cid=%02x, status=%x\r\n", conn_id, status);
}

static void sle_conn_register_cbks(void) {
    sle_connection_callbacks_t conn_cbks = {0};
    conn_cbks.connect_state_changed_cb = sle_connect_state_changed_cbk;
    conn_cbks.pair_complete_cb = sle_pair_complete_cbk;
    sle_connection_register_callbacks(&conn_cbks);
}

/*============================================================================*/
/*====================== Initialization & Startup Flow =======================*/
/*============================================================================*/
static void *adxl355_server_task(const char *arg);
static void sle_enable_cbk(uint8_t status);
static void sle_power_on_cbk(uint8_t status);


/**
 * @brief Main application task.
 * This task initializes all application modules.
 */
static void *adxl355_server_task(const char *arg)
{
    unused(arg);
    
    osal_printk("[SERVER] Main Task Started. Waiting for SLE stack to be enabled...\r\n");
    // Wait until the SLE stack is ready.
    osSemaphoreAcquire(g_sle_enabled_sem, osWaitForever);
    osal_printk("[SERVER] SLE stack enabled. Starting application initialization.\r\n");

    g_device_unique_id = g_sle_server_mac[SLE_ADDR_LEN - 1];
    osal_printk("[SERVER] Unique ID is 0x%02X.\r\n", g_device_unique_id);

    g_send_data_sem = osSemaphoreNew(1, 0, NULL);
    if (g_send_data_sem == NULL) {
        osal_printk("[SERVER] FATAL: Failed to create send data semaphore.\r\n");
        return NULL;
    }
    
    g_drdy_sem = osSemaphoreNew(1, 0, NULL);
    if (g_drdy_sem == NULL) {
        osal_printk("[SERVER] FATAL: Failed to create DRDY semaphore.\r\n");
        return NULL;
    }

    // Register application-specific callbacks
    sle_conn_register_cbks();
    sle_ssaps_register_cbks();
    
    // Create SLE services and properties
    sle_uuid_server_add();

    // Initialize hardware peripherals
    ADXL355_HardwareInit();
    ADXL355_DRDY_Init();

    // Create worker tasks
    osal_task *task_handle = NULL;
    osal_kthread_lock();

    task_handle = osal_kthread_create((osal_kthread_handler)sensor_sampling_task, 0, "SensorSamplingTask", SENSOR_SAMPLING_TASK_STACK_SIZE);
    if (task_handle) {
        osal_kthread_set_priority(task_handle, SENSOR_SAMPLING_TASK_PRIO);
        osal_kfree(task_handle);
    }

    task_handle = osal_kthread_create((osal_kthread_handler)data_sending_task, 0, "DataSendingTask", DATA_SENDING_TASK_STACK_SIZE);
    if (task_handle) {
        osal_kthread_set_priority(task_handle, DATA_SENDING_TASK_PRIO);
        osal_kfree(task_handle);
    }

    osal_kthread_unlock();

    // Start advertising
    sle_adxl355_server_adv_init();

    osal_printk("[SERVER] Initialization complete. System is running.\r\n");

    // This main task has completed its setup. It can now idle or be deleted.
    while(1) {
        osal_msleep(10000); // Idle loop
    }

    return NULL;
}

/**
 * @brief Callback invoked when the SLE stack is enabled.
 */
static void sle_enable_cbk(uint8_t status)
{
    osal_printk("SLE enable callback, status: %d\r\n", status);
    if (status == ERRCODE_SLE_SUCCESS) {
        // Signal the main task that the stack is ready
        osSemaphoreRelease(g_sle_enabled_sem);
    }
}

/**
 * @brief Callback invoked when the SLE hardware is powered on.
 */
static void sle_power_on_cbk(uint8_t status)
{
    osal_printk("SLE power on callback, status: %d\r\n", status);
    if (status == ERRCODE_SLE_SUCCESS) {
        enable_sle();
    }
}


/**
 * @brief Main entry point for the application.
 */
static void sle_adxl355_entry(void)
{
    // Create the synchronization semaphore before anything else
    g_sle_enabled_sem = osSemaphoreNew(1, 0, NULL);
    if (g_sle_enabled_sem == NULL) {
        osal_printk("[SERVER] FATAL: Failed to create SLE enabled semaphore.\r\n");
        return;
    }

    // Register callbacks that will be triggered by the SLE stack
    sle_dev_manager_callbacks_t dev_mgr_cbks = {0};
    dev_mgr_cbks.sle_power_on_cb = sle_power_on_cbk;
    dev_mgr_cbks.sle_enable_cb = sle_enable_cbk;
    sle_dev_manager_register_callbacks(&dev_mgr_cbks);
    
    // Create the main application task
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)adxl355_server_task, 0, "ADXL355SrvMain", MAIN_TASK_STACK_SIZE);
    if (task_handle) {
        osal_kthread_set_priority(task_handle, MAIN_TASK_PROI);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

app_run(sle_adxl355_entry);
