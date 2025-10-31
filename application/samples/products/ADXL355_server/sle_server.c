/**
 * @file sle_server.c
 * @brief Merged and Refactored SLE server for ADXL355 data transmission.
 * @version 2.2 (Modified)
 * @date 2023-10-18
 *
 */

// Version : 2.3
// use g_sample_counter instead of transmission counter

/*============================================================================*/
/*============================= Include Headers ==============================*/
/*============================================================================*/
#include "common_def.h"
#include "securec.h"
#include <stdio.h>
#include <string.h>

// SLE/OS Headers
#include "app_init.h"
#include "osal_task.h"
#include "osal_debug.h"
#include "osal_addr.h"
#include "tcxo.h"
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_device_manager.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_server.h"
#include "interrupt.h" /* IMPORTANT: Include for uapi_irq_lock/restore */
#include "clocks_switch.h"

// Hardware Driver Headers
#include "pinctrl.h"
#include "gpio.h"
#include "i2c.h"
#include "watchdog.h" 

// Local Merged Header
#include "sle_server.h"

// Memory debugging - include LiteOS memory functions
#include "los_memory.h"

/*============================================================================*/
/*======================== Configuration Parameters ==========================*/
/*============================================================================*/

// --- 设备MAC地址配置 ---
#define SERVER_MAC_ADDRESS              { 0x11, 0x22, 0x33, 0x44, 0x55, 0x08 }

// --- Task & System Configuration ---
#define ADXL355_SERVER_TASK_STACK_SIZE    0x2000  // Main task stack
#define SENSOR_SAMPLING_TASK_STACK_SIZE   0x1000  // Task for reading sensor data
#define DATA_SENDING_TASK_STACK_SIZE      0x2000  // Task for sending data via SLE
#define SENSOR_SAMPLING_TASK_PRIO         25      // High priority to not miss interrupts
#define DATA_SENDING_TASK_PRIO            25
#define ADXL355_SERVER_TASK_PRIO          26      // Low priority for setup task

// --- Message Queue Configuration ---
#define MSG_QUEUE_LEN                     5
#define MSG_QUEUE_MAX_SIZE                4       // Size of a dummy message (e.g., uint32_t)
#define MSG_QUEUE_WAIT_FOREVER            0xFFFFFFFF

// --- Logging ---
#define SLE_SERVER_LOG   "[ADXL355_SERVER]"
#define sample_at_log_print(fmt, args...) osal_printk(fmt, ##args)

// --- SLE Advertising Parameters (Aligned with WS63) ---
#define NAME_MAX_LENGTH                       16
#define SLE_CONN_INTV_MIN_DEFAULT             0x54    
#define SLE_CONN_INTV_MAX_DEFAULT             0x54    
#define SLE_ADV_INTERVAL_MIN_DEFAULT          0xC8    // 25ms (200 * 125us)
#define SLE_ADV_INTERVAL_MAX_DEFAULT          0xC8    
#define SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT  0x64   // 100 * 10ms = 1s   
#define SLE_CONN_MAX_LATENCY                  0x00
#define SLE_ADV_TX_POWER                      10      
#define SLE_ADV_HANDLE_DEFAULT                1
#define SLE_ADV_DATA_LEN_MAX                  251
#define DEFAULT_SLE_SPEED_MTU_SIZE            251     

// --- Staged Startup Control (Aligned with WS63) ---
#define INITIAL_COLLECTION_START_COUNT        2
#define INITIAL_TRANSMISSION_START_COUNT      3

// --- Data Framing Definitions (Aligned with WS63/Client) ---
#define MTU_PACKET_END_FLAG               0xB4 // Trailer for a standard MTU packet
#define TRANSMISSION_END_FLAG             0x4B // Special trailer for the final MTU packet
#define MTU_HEADER_SIZE                   8    // total_pkts(2) + current_pkt(2) + timestamp(4)
#define MTU_TRAILER_SIZE                  1    // end_flag(1)
#define ATT_HEADER_OVERHEAD               3
#define MTU_OVERHEAD_SIZE                 (MTU_HEADER_SIZE + MTU_TRAILER_SIZE)
#define MTU_PAYLOAD_SIZE                  (DEFAULT_SLE_SPEED_MTU_SIZE - MTU_OVERHEAD_SIZE - ATT_HEADER_OVERHEAD)

// --- 设备状态定义 ---
#define DEVICE_STATUS_NO_DATA             0x00
#define DEVICE_STATUS_HAS_DATA            0x01

// --- 看门狗配置 ---
#define WDT_SYSTEM_TIMEOUT_S              30  // 修改：系统看门狗超时时间 (秒)
#define WDT_RESET_MODE                    1   // 看门狗模式：超时后复位系统
#define ULP_WDT_SYSTEM_TIMEOUT_S          2     // 低功耗看门狗超时时间 

/*============================================================================*/
/*========================= ADXL355 Driver Defines ===========================*/
/*============================================================================*/
// I2C configuration
#define ADXL355_I2C_BUS_ID      CONFIG_I2C_MASTER_BUS_ID
#define ADXL355_I2C_BAUDRATE    400000   // 400kHz
#define ADXL355_I2C_ADDR        0x1D

// ADXL355 registers addresses
#define ADXL355_DEVID_AD        0x00
#define ADXL355_PARTID          0x02
#define ADXL355_XDATA3          0x08
#define ADXL355_FILTER          0x28
#define ADXL355_RANGE           0x2C
#define ADXL355_POWER_CTL       0x2D

// ODR and Range values
#define ADXL355_ODR_4000                  ((uint16_t)0x00)
#define ADXL355_ODR_2000                  ((uint16_t)0x01)
#define ADXL355_ODR_1000                  ((uint16_t)0x02)
#define ADXL355_ODR_500                   ((uint16_t)0x03)
#define ADXL355_ODR_250                   ((uint16_t)0x04)
#define ADXL355_ODR_125                   ((uint16_t)0x05)
#define ADXL355_ODR_62_5                  ((uint16_t)0x06)
#define ADXL355_ODR_31_25                 ((uint16_t)0x07)
#define ADXL355_ODR_15_625                ((uint16_t)0x08)
#define ADXL355_ODR_7_813                 ((uint16_t)0x09)
#define ADXL355_ODR_3_906                 ((uint16_t)0x0A)

#define ADXL355_RANGE_2G        ((uint16_t)0x81)
#define ADXL355_RANGE_2G_SCALE  256000.0f

// Sensor Data Ring Buffer Configuration
#define SENSOR_DATA_BUFFER_SIZE       1200 // Buffer size in bytes
#define SENSOR_SAMPLE_SIZE_BYTES      (3 * sizeof(int16_t))
#define SENSOR_BUFFER_SAMPLES         (SENSOR_DATA_BUFFER_SIZE / SENSOR_SAMPLE_SIZE_BYTES) //1200 / 6 = 200 samples
/*============================================================================*/
/*======================= SLE Service & Property UUIDs =======================*/
/*============================================================================*/
#define UUID_LEN_2 2
#define SLE_UUID_SERVER_SERVICE         0xABCD
#define SLE_UUID_SERVER_NTF_REPORT      0x1122
#define SLE_UUID_SERVER_CMD_CONTROL     0x1123
#define SLE_UUID_SERVER_STATUS_REPORT   0x1124
#define SLE_UUID_SERVER_TX_COUNT_REPORT 0x1125 // NEW: Dedicated UUID for transmission counter

#define SLE_UUID_TEST_PROPERTIES (SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE)
#define SLE_UUID_TEST_DESCRIPTOR (SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE)
#define SLE_UUID_TEST_OPERATION_INDICATION (SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE)

/*============================================================================*/
/*============================ Global Variables ==============================*/
/*============================================================================*/
// --- SLE and Task Control ---
static bool g_is_sle_connected = false;
static bool g_has_been_connected_once = false; // 新增：用于看门狗逻辑
static unsigned long g_drdy_msg_queue_id;
static unsigned long g_send_data_msg_queue_id;

static volatile bool g_is_sending_data = false;
static uint16_t g_sle_conn_hdl = 0;
static uint8_t g_server_id = 0;
static uint16_t g_service_handle = 0;
static uint16_t g_property_handle = 0;
static uint16_t g_command_property_handle = 0;
static uint16_t g_status_property_handle = 0;
static uint16_t g_tx_count_property_handle = 0; // NEW: Handle for the new characteristic
static char g_sle_uuid_app_uuid[UUID_LEN_2] = { 0x12, 0x34 };

static osal_task *sender_task_handle = NULL;

// --- Staged Startup Control ---
static uint32_t g_start_command_count = 0;
static volatile bool g_collection_phase_started = false;
static volatile bool g_is_operational = false;

// --- ADXL355 Data ---
static int16_t volatile i16SensorX, i16SensorY, i16SensorZ;

// counter
static volatile uint32_t g_sample_counter = 0;
static volatile uint32_t g_transmission_counter = 0;
static volatile int32_t g_debut = 0; // NEW: Total transmission sessions

// --- Sensor Data Ring Buffer ---
static int16_t g_sensor_data_buffer[SENSOR_BUFFER_SAMPLES * 3];
static volatile uint32_t g_sensor_buffer_head = 0;
static volatile uint32_t g_sensor_buffer_tail = 0;

// --- Advertising Data ---
static uint8_t g_sle_local_name[NAME_MAX_LENGTH] = "sle_adxl_1";

// --- Transmission Buffer ---
static uint8_t tx_linear_buffer[SENSOR_DATA_BUFFER_SIZE];

static uint32_t freq;

/*============================================================================*/
/*========================= Function Prototypes ==============================*/
/*============================================================================*/
static void *sensor_sampling_task(void *arg);
static void *data_sending_task(void *arg);
static void dequeue_and_send_sensor_data(void);
static errcode_t sle_uart_server_send_report_by_handle(const uint8_t *data, uint16_t len);
static errcode_t sle_uart_server_send_status_report(uint8_t status);
// static errcode_t sle_uart_server_send_transmission_count(uint8_t count); // NEW
void adxl355_drdy_cb(pin_t pin, uintptr_t param);


/*============================================================================*/
/*======================= ADXL355 Driver Implementation ======================*/
/*============================================================================*/
static void ADXL355_WriteRegister(uint8_t reg, uint8_t val)
{
    i2c_data_t data = {0};
    uint8_t tx[2] = {reg, val};
    data.send_buf = tx;
    data.send_len = 2;
    data.receive_buf = NULL;
    data.receive_len = 0;
    uapi_i2c_master_write(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
}

static uint32_t ADXL355_ReadRegister(uint8_t reg, adxl355_reglen_t len)
{
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

static void ADXL355_Standby(void)
{
    uint8_t pwr = (uint8_t)ADXL355_ReadRegister(ADXL355_POWER_CTL, ADXL355_REGS_1);
    pwr |= 0x01;
    ADXL355_WriteRegister(ADXL355_POWER_CTL, pwr);
}

static void ADXL355_Startup(void)
{
    uint8_t pwr = (uint8_t)ADXL355_ReadRegister(ADXL355_POWER_CTL, ADXL355_REGS_1);
    pwr &= ~0x01;
    ADXL355_WriteRegister(ADXL355_POWER_CTL, pwr);
}

// static int16_t ADXL355_Convert(uint32_t raw)
// {
//     int32_t signed_20_bit_val;
//     raw = (raw >> 4) & 0x000FFFFF;

//     if (raw & 0x00080000) {
//         signed_20_bit_val = (int32_t)(raw | 0xFFF00000);
//     } else {
//         signed_20_bit_val = (int32_t)raw;
//     }
//     return (int16_t)(signed_20_bit_val >> 4);
// }

static int16_t ADXL355_Convert(uint32_t raw)
{
    uint32_t raw_20bit = (raw >> 4) & 0x000FFFFF;
    uint32_t sign_bit_component = raw_20bit & 0x00080000;
    uint32_t lower_15_bits = raw_20bit & 0x00007FFF;
    uint32_t new_sign_bit = sign_bit_component >> 4;
    uint32_t combined_16bit = new_sign_bit | lower_15_bits;
    if (combined_16bit & 0x8000) {
        return (int16_t)(combined_16bit | 0xFFFF0000);
    } else {
        return (int16_t)combined_16bit;
    }
}

static void ADXL355_ReadData(void)
{
    uint8_t buf[9];
    i2c_data_t data = {0};
    uint8_t reg = ADXL355_XDATA3;

    data.send_buf = &reg;
    data.send_len = 1;
    data.receive_buf = buf;
    data.receive_len = 9;

    uapi_i2c_master_write(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);
    uapi_i2c_master_read(ADXL355_I2C_BUS_ID, ADXL355_I2C_ADDR, &data);

    uint32_t ui32SensorX = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    uint32_t ui32SensorY = (buf[3] << 16) | (buf[4] << 8) | buf[5];
    uint32_t ui32SensorZ = (buf[6] << 16) | (buf[7] << 8) | buf[8];

    i16SensorX = ADXL355_Convert(ui32SensorX);
    i16SensorY = ADXL355_Convert(ui32SensorY);
    i16SensorZ = ADXL355_Convert(ui32SensorZ);
}

static void adxl355_init(void) {
    if (ADXL355_I2C_BUS_ID == 0) {
        uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, HAL_PIO_I2C0_CLK);
        uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, HAL_PIO_I2C0_DATA);
    } else {
        uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, HAL_PIO_I2C1_CLK);
        uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, HAL_PIO_I2C1_DATA);
    }
    uapi_pin_set_pull(CONFIG_I2C_SCL_MASTER_PIN, PIN_PULL_UP);
    uapi_pin_set_pull(CONFIG_I2C_SDA_MASTER_PIN, PIN_PULL_UP);
    uapi_tcxo_delay_ms(10);
    errcode_t ret = uapi_i2c_master_init(ADXL355_I2C_BUS_ID, ADXL355_I2C_BAUDRATE, 0);
    if (ret != EOK) {
        // sample_at_log_print("%s I2C Init failed\r\n", SLE_SERVER_LOG);
        return;
    }
    else {
        // sample_at_log_print("%s I2C Init success\r\n", SLE_SERVER_LOG);
    }

    ADXL355_Standby();
    ADXL355_WriteRegister(ADXL355_RANGE, ADXL355_RANGE_2G);
    ADXL355_WriteRegister(ADXL355_FILTER, ADXL355_ODR_62_5);
    ADXL355_Startup();

    uapi_pin_set_mode(ADXL355_DRDY_PIN, ADXL355_DRDY_FUNC);
    uapi_gpio_set_dir(ADXL355_DRDY_PIN, GPIO_DIRECTION_INPUT);
    uapi_gpio_register_isr_func(ADXL355_DRDY_PIN, GPIO_INTERRUPT_RISING_EDGE, (gpio_callback_t)adxl355_drdy_cb);
    uapi_gpio_enable_interrupt(ADXL355_DRDY_PIN);
    // sample_at_log_print("%s ADXL355 Initialized.\r\n", SLE_SERVER_LOG);
}


/* Memory debugging helper function */
static void print_memory_info(const char* tag) {
    extern UINT8 *m_aucSysMem0;
    LOS_MEM_POOL_STATUS status;
    if (LOS_MemInfoGet(m_aucSysMem0, &status) == LOS_OK) {
        osal_printk("[MEM-%s] Total=%u, Used=%u, Free=%u\r\n",
            tag,
            status.uwTotalUsedSize + status.uwTotalFreeSize,
            status.uwTotalUsedSize,
            status.uwTotalFreeSize);
    }
}


/*============================================================================*/
/*========================== SLE Utility Functions ===========================*/
/*============================================================================*/
static void encode2byte_little(uint8_t *_ptr, uint16_t data)
{
    *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 0x8);
    *(uint8_t *)(_ptr) = (uint8_t)(data);
}

static void sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    static uint8_t sle_uuid_base[] = { 0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA, \
                                       0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    errno_t ret = memcpy_s(out->uuid, SLE_UUID_LEN, sle_uuid_base, SLE_UUID_LEN);
    if (ret != EOK) {
        // sample_at_log_print("%s UUID memcpy failed\r\n", SLE_SERVER_LOG);
        return;
    }
    out->len = UUID_LEN_2;
    encode2byte_little(&out->uuid[14], u2);
}

/*============================================================================*/
/*=========================== Advertising Setup ==============================*/
/*============================================================================*/
static uint16_t sle_set_adv_local_name(uint8_t *adv_data, uint16_t max_len)
{
    errno_t ret;
    uint8_t index = 0;
    uint8_t local_name_len = (uint8_t)strlen((char *)g_sle_local_name);

    if (local_name_len + 2 > max_len) return 0;

    adv_data[index++] = local_name_len + 1;
    adv_data[index++] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    ret = memcpy_s(&adv_data[index], max_len - index, g_sle_local_name, local_name_len);
    if (ret != EOK) {
        // sample_at_log_print("%s Set name memcpy failed\r\n", SLE_SERVER_LOG);
        return 0;
    }
    return (uint16_t)index + local_name_len;
}

static uint16_t sle_set_adv_data(uint8_t *adv_data, uint16_t max_len)
{
    uint16_t idx = 0;
    struct sle_adv_common_value { uint8_t length; uint8_t type; uint8_t value; };
    size_t len = sizeof(struct sle_adv_common_value);

    if (idx + len > max_len) return idx;
    struct sle_adv_common_value adv_disc_level = {
        .length = (uint8_t)(len - 1), .type = SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL, .value = SLE_ANNOUNCE_LEVEL_NORMAL,
    };
    errno_t ret1 = memcpy_s(&adv_data[idx], max_len - idx, &adv_disc_level, len);
    if (ret1 != EOK) {
        // sample_at_log_print("%s Set adv_disc_level memcpy failed\r\n", SLE_SERVER_LOG);
        return idx;
    }
    idx += len;

    if (idx + len > max_len) return idx;
    struct sle_adv_common_value adv_access_mode = {
        .length = (uint8_t)(len - 1), .type = SLE_ADV_DATA_TYPE_ACCESS_MODE, .value = 0,
    };
    errno_t ret2 = memcpy_s(&adv_data[idx], max_len - idx, &adv_access_mode, len);
    if (ret2 != EOK) {
        // sample_at_log_print("%s Set adv_access_mode memcpy failed\r\n", SLE_SERVER_LOG);
        return idx;
    }
    idx += len;

    return idx;
}

static uint16_t sle_set_scan_response_data(uint8_t *scan_rsp_data, uint16_t max_len)
{
    uint16_t idx = 0;
    struct sle_adv_common_value { uint8_t length; uint8_t type; uint8_t value; };
    size_t scan_rsp_data_len = sizeof(struct sle_adv_common_value);

    if (idx + scan_rsp_data_len > max_len) return idx;
    struct sle_adv_common_value tx_power_level = {
        .length = (uint8_t)(scan_rsp_data_len - 1), .type = SLE_ADV_DATA_TYPE_TX_POWER_LEVEL, .value = SLE_ADV_TX_POWER,
    };
    errno_t ret = memcpy_s(scan_rsp_data, max_len, &tx_power_level, scan_rsp_data_len);
    if (ret != EOK) {
        // sample_at_log_print("%s Set tx_power_level memcpy failed\r\n", SLE_SERVER_LOG);
        return idx;
    }
    idx += scan_rsp_data_len;

    idx += sle_set_adv_local_name(&scan_rsp_data[idx], max_len - idx);
    return idx;
}

static int sle_set_default_announce_param(void)
{
    sle_announce_param_t param = {0};
    uint8_t local_addr[SLE_ADDR_LEN] = SERVER_MAC_ADDRESS;

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
    errno_t ret = memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN);
    if (ret != EOK) {
        // sample_at_log_print("%s Set own_addr memcpy failed\r\n", SLE_SERVER_LOG);
        return -1;
    }

    return sle_set_announce_param(param.announce_handle, &param);
}

static int sle_set_default_announce_data(void)
{
    sle_announce_data_t data = {0};
    uint8_t adv_handle = SLE_ADV_HANDLE_DEFAULT;
    static uint8_t announce_data[SLE_ADV_DATA_LEN_MAX] = {0};
    static uint8_t seek_rsp_data[SLE_ADV_DATA_LEN_MAX] = {0};

    data.announce_data_len = sle_set_adv_data(announce_data, SLE_ADV_DATA_LEN_MAX);
    data.announce_data = announce_data;

    data.seek_rsp_data_len = sle_set_scan_response_data(seek_rsp_data, SLE_ADV_DATA_LEN_MAX);
    data.seek_rsp_data = seek_rsp_data;

    return sle_set_announce_data(adv_handle, &data);
}

static void sle_uart_server_adv_init(void)
{
    // sample_at_log_print("%s ADV init start\r\n", SLE_SERVER_LOG);
    (void)sle_set_default_announce_param();
    (void)sle_set_default_announce_data();
    (void)sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
    // sample_at_log_print("%s ADV init complete, starting announce\r\n", SLE_SERVER_LOG);
}

/*============================================================================*/
/*==================== Application Core Logic & Tasks ========================*/
/*============================================================================*/

void adxl355_drdy_cb(pin_t pin, uintptr_t param) {
    (void)pin;
    (void)param;


    if (g_collection_phase_started) {
        uint32_t dummy_msg = 1;
        (void)osal_msg_queue_write_copy(g_drdy_msg_queue_id, &dummy_msg, sizeof(dummy_msg), 0);
        g_sample_counter++; // Increment sample counter
    }

}

static void *sensor_sampling_task(void *arg) {
    (void)arg;
    uint32_t dummy_msg;
    uint32_t msg_size = sizeof(dummy_msg);
    // sample_at_log_print("%s Sensor Sampling Task started.\r\n", SLE_SERVER_LOG);

    while (1) {
        if (!g_collection_phase_started) {
            osal_msleep(500);
            continue;
        }

        if (osal_msg_queue_read_copy(g_drdy_msg_queue_id, &dummy_msg, &msg_size, MSG_QUEUE_WAIT_FOREVER) == OSAL_SUCCESS) {
            if (!g_collection_phase_started) continue;

            uapi_gpio_disable_interrupt(ADXL355_DRDY_PIN);

            ADXL355_ReadData();

            uint32_t head = g_sensor_buffer_head;
            uint32_t tail = g_sensor_buffer_tail;
            uint32_t next_tail = (tail + 1) % SENSOR_BUFFER_SAMPLES;

            if (next_tail != head) {
                uint32_t index = tail * 3;
                g_sensor_data_buffer[index] = i16SensorX;
                g_sensor_data_buffer[index + 1] = i16SensorY;
                g_sensor_data_buffer[index + 2] = i16SensorZ;
                g_sensor_buffer_tail = next_tail;
            }
            uapi_gpio_enable_interrupt(ADXL355_DRDY_PIN);
        }
    }
    return NULL;
}

static void *data_sending_task(void *arg) {
    (void)arg;
    uint32_t dummy_msg;
    uint32_t msg_size = sizeof(dummy_msg);
    // sample_at_log_print("%s Data Sending Task started.\r\n", SLE_SERVER_LOG);
    while (1) {
        if (osal_msg_queue_read_copy(g_send_data_msg_queue_id, &dummy_msg, &msg_size, MSG_QUEUE_WAIT_FOREVER) == OSAL_SUCCESS) {
            g_transmission_counter++;
            // osal_printk("%s Trans Count: %u\r\n", "[Count]", g_transmission_counter);
            dequeue_and_send_sensor_data();    
        }
    }
    return NULL;
}

void dequeue_and_send_sensor_data(void) {
    if (!g_is_sle_connected || g_is_sending_data) return;
    g_is_sending_data = true;


    uint32_t total_bytes_to_send = 0;

    uint32_t irq_status = osal_irq_lock();
    uint32_t head = g_sensor_buffer_head;
    uint32_t tail = g_sensor_buffer_tail;

    if (head != tail) {
        uint32_t samples_available;
        if (tail > head) {
            samples_available = tail - head;
        } else {
            samples_available = SENSOR_BUFFER_SAMPLES - head + tail;
            
        }
        g_debut+= g_sample_counter-samples_available; // NEW: Update total transmission sessions
        // osal_printk("%s Buffer wrap-around detected. Head: %u, Tail: %u, Samples: %u, debut: %u\r\n", SLE_SERVER_LOG, head, tail, g_sample_counter, g_debut);
        
        // sle_uart_server_send_transmission_count((uint32_t)g_sample_counter);
        // osal_msleep(10);
        g_sample_counter = 0; // Reset counter

        total_bytes_to_send = samples_available * SENSOR_SAMPLE_SIZE_BYTES;

        if (total_bytes_to_send > 0 && total_bytes_to_send <= SENSOR_DATA_BUFFER_SIZE) {
            if (tail > head) {
                if (memcpy_s(tx_linear_buffer, sizeof(tx_linear_buffer), &g_sensor_data_buffer[head * 3], total_bytes_to_send) != EOK) {
                    // sample_at_log_print("%s Data memcpy failed\r\n", SLE_SERVER_LOG);
                    total_bytes_to_send = 0;
                }
            } else {
                uint32_t part1_bytes = (SENSOR_BUFFER_SAMPLES - head) * SENSOR_SAMPLE_SIZE_BYTES;
                if (memcpy_s(tx_linear_buffer, sizeof(tx_linear_buffer), &g_sensor_data_buffer[head * 3], part1_bytes) != EOK) {
                    // sample_at_log_print("%s Data part1 memcpy failed\r\n", SLE_SERVER_LOG);
                    total_bytes_to_send = 0;
                }

                uint32_t part2_bytes = total_bytes_to_send - part1_bytes;
                if (part2_bytes > 0) {
                    if (memcpy_s(&tx_linear_buffer[part1_bytes], sizeof(tx_linear_buffer) - part1_bytes, g_sensor_data_buffer, part2_bytes) != EOK) {
                        // sample_at_log_print("%s Data part2 memcpy failed\r\n", SLE_SERVER_LOG);
                        total_bytes_to_send = 0;
                    }
                }
            }
        } else {
            total_bytes_to_send = 0;
        }
        g_sensor_buffer_head = tail;
        // osal_printk("%s Head moved to %u, Head at %u, Tail at %u\r\n", SLE_SERVER_LOG, g_sensor_buffer_head, head, tail);
    }
    osal_irq_restore(irq_status);
    
    // MODIFIED: Send status and transmission count over separate characteristics
    uint8_t current_status = (total_bytes_to_send > 0) ? DEVICE_STATUS_HAS_DATA : DEVICE_STATUS_NO_DATA;
    sle_uart_server_send_status_report(current_status);
    osal_msleep(10);

    if (total_bytes_to_send > 0) {
        // sample_at_log_print("%s Dequeued %u bytes to send.\r\n", SLE_SERVER_LOG, total_bytes_to_send);
        static uint8_t packet_buffer[DEFAULT_SLE_SPEED_MTU_SIZE];
        uint16_t total_packets = (total_bytes_to_send + MTU_PAYLOAD_SIZE - 1) / MTU_PAYLOAD_SIZE;
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
            *p++ = (uint8_t)(packet_count >> 8);  *p++ = (uint8_t)(packet_count & 0xFF);
            uint32_t timestamp = (uint32_t)uapi_tcxo_get_ms();
            *p++ = (uint8_t)(timestamp >> 24); *p++ = (uint8_t)(timestamp >> 16);
            *p++ = (uint8_t)(timestamp >> 8);  *p++ = (uint8_t)(timestamp & 0xFF);

            if (memcpy_s(p, sizeof(packet_buffer) - MTU_HEADER_SIZE, &tx_linear_buffer[sent_bytes], bytes_for_this_payload) != EOK) {
                // sample_at_log_print("%s Payload memcpy failed for packet %u\r\n", SLE_SERVER_LOG, packet_count);
                break;
            }
            p += bytes_for_this_payload;

            *p = (packet_count == total_packets) ? TRANSMISSION_END_FLAG : MTU_PACKET_END_FLAG;

            if (g_debut < 0) {
                return;
            }

            if (sle_uart_server_send_report_by_handle(packet_buffer, total_packet_size) != ERRCODE_SLE_SUCCESS) {
                // sample_at_log_print("%s Failed to send packet %u.\r\n", SLE_SERVER_LOG, packet_count);
                break;
            }
            else {
                // sample_at_log_print("%s Sent packet %u/%u, %u bytes.\r\n", SLE_SERVER_LOG, packet_count, total_packets, bytes_for_this_payload);
            }
            sent_bytes += bytes_for_this_payload;
            osal_msleep(10);
        }
    } else {
        uint8_t empty_packet[MTU_HEADER_SIZE + MTU_TRAILER_SIZE];
        uint8_t* p = empty_packet;
        *p++ = 0; *p++ = 1;
        *p++ = 0; *p++ = 1;
        uint32_t timestamp = (uint32_t)uapi_tcxo_get_ms();
        *p++ = (uint8_t)(timestamp >> 24); *p++ = (uint8_t)(timestamp >> 16);
        *p++ = (uint8_t)(timestamp >> 8); *p++ = (uint8_t)(timestamp & 0xFF);
        *p = TRANSMISSION_END_FLAG;
        errcode_t ret = sle_uart_server_send_report_by_handle(empty_packet, sizeof(empty_packet));
        if (ret != ERRCODE_SLE_SUCCESS) {
            // sample_at_log_print("%s Failed to send empty packet.\r\n", SLE_SERVER_LOG);
        } else {
            // sample_at_log_print("%s Sent empty packet to indicate no data.\r\n", SLE_SERVER_LOG);
        }
    }

    g_is_sending_data = false;
}

/*============================================================================*/
/*============================ SLE SSAPS Callbacks ===========================*/
/*============================================================================*/
static void ssaps_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    sample_at_log_print("%s Read request: server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
    // osal_printk("ssaps_read_request_cbk WAS CALLED !!!\r\n");
}

static void ssaps_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
    errcode_t status)
{
    (void)server_id; (void)conn_id;
    if (status != ERRCODE_SLE_SUCCESS || write_cb_para == NULL || write_cb_para->value == NULL) return;

    if (write_cb_para->handle == g_command_property_handle) {
        uint8_t command = write_cb_para->value[0];
        if (command == 0x01) {
            if (g_is_operational) {
                // sample_at_log_print("%s Received START command. Triggering data send.\r\n", SLE_SERVER_LOG);
                uint32_t dummy_msg = 1;
                (void)osal_msg_queue_write_copy(g_send_data_msg_queue_id, &dummy_msg, sizeof(dummy_msg), 0);
                return;
            }

            g_start_command_count++;
            // sample_at_log_print("%s Received initial START command (%u/%u to send).\r\n", SLE_SERVER_LOG, g_start_command_count, INITIAL_TRANSMISSION_START_COUNT);

            if (!g_collection_phase_started && g_start_command_count >= INITIAL_COLLECTION_START_COUNT) {
                // sample_at_log_print("%s Collection threshold reached. Starting data sampling.\r\n", SLE_SERVER_LOG);
                g_collection_phase_started = true;
            }
            if (!g_is_operational && g_start_command_count >= INITIAL_TRANSMISSION_START_COUNT) {
                // sample_at_log_print("%s Transmission threshold reached. System is now fully operational.\r\n", SLE_SERVER_LOG);
                g_is_operational = true;
                uint32_t dummy_msg = 1;
                (void)osal_msg_queue_write_copy(g_send_data_msg_queue_id, &dummy_msg, sizeof(dummy_msg), 0);
            }
        }
    }
}

static void sle_ssaps_set_info(void)
{
    ssap_exchange_info_t info = { .mtu_size = DEFAULT_SLE_SPEED_MTU_SIZE, .version = 1 };
    ssaps_set_info(g_server_id, &info);
    // sample_at_log_print("%s MTU info set to %u.\r\n", SLE_SERVER_LOG, info.mtu_size);
}

static void ssaps_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id,  ssap_exchange_info_t *mtu_size,
    errcode_t status)
{
    sample_at_log_print("%s MTU changed: server_id:%x, conn_id:%x, mtu_size:%x, status:%x\r\n",
        SLE_SERVER_LOG, server_id, conn_id, mtu_size->mtu_size, status);
    sle_ssaps_set_info();
}

static void sle_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t reason)
{
    (void)addr; (void)pair_state; (void)reason;
    g_sle_conn_hdl = conn_id;

    // 根据图片中的逻辑，定义并填充连接参数结构体
    sle_connection_param_update_t conn_param = {0};
    conn_param.conn_id = conn_id;
    conn_param.interval_min = SLE_CONN_INTV_MIN_DEFAULT;
    conn_param.interval_max = SLE_CONN_INTV_MAX_DEFAULT;
    conn_param.max_latency = SLE_CONN_MAX_LATENCY;
    conn_param.supervision_timeout = SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT;

    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        // 在连接建立后，发起连接参数更新请求
        sle_update_connect_param(&conn_param);

        g_is_sle_connected = true;
        g_has_been_connected_once = true; // 新增：标记已经连接过一次
        g_collection_phase_started = false;
        g_is_operational = false;
        g_start_command_count = 0;

        (void)uapi_watchdog_init(WDT_SYSTEM_TIMEOUT_S);
        (void)uapi_watchdog_enable((wdt_mode_t)WDT_RESET_MODE);

        // sample_at_log_print("%s Device connected. Watchdog kicking will resume. Waiting for initial commands.\r\n", SLE_SERVER_LOG);

    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        g_is_sle_connected = false;
        g_collection_phase_started = false;
        g_is_operational = false;
        
        uint32_t irq_status = osal_irq_lock();
        int  i = 0;
        for (i = 0; i < 600; i++) {
            g_sensor_data_buffer[i] = 0;
        }
        int j = 0;
        for (j = 0; j < 1200; j++) {
            tx_linear_buffer[j] = 0;
        }
        g_sensor_buffer_head = 0;
        g_sensor_buffer_tail = 0;
        g_sample_counter = 0;
        g_transmission_counter = 0;
        osal_irq_restore(irq_status);

        (void)uapi_watchdog_deinit();

        // sample_at_log_print("%s Device disconnected. Watchdog kicking paused. Buffers and state reset.\r\n", SLE_SERVER_LOG);
        // (void)sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
    }
}

// =================================================================================
// ==================== NEW FUNCTIONS ADDED FROM WS63 V2.4 =====================
// =================================================================================
/**
 * @brief Callback for handling connection parameter update events.
 */
static void sle_sample_update_cbk(uint16_t conn_id, errcode_t status, const sle_connection_param_update_evt_t *param)
{
    (void)status;
    sample_at_log_print("%s Conn param updated: conn_id=%d, interval=0x%02x\r\n", SLE_SERVER_LOG, conn_id, param->interval);
}

/**
 * @brief Callback for handling connection parameter update requests from the client.
 */
static void sle_sample_update_req_cbk(uint16_t conn_id, errcode_t status, const sle_connection_param_update_req_t *param)
{
    (void)conn_id;
    (void)status;
    sample_at_log_print("%s Conn param update req: interval_min=0x%02x, interval_max=0x%02x\r\n",
           SLE_SERVER_LOG, param->interval_min, param->interval_max);
}
// =================================================================================
// ==================== END OF NEWLY ADDED FUNCTIONS ===============================
// =================================================================================

static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    (void)addr;
    sample_at_log_print("%s Pair complete: conn_id=%02x, status=%x\r\n", SLE_SERVER_LOG, conn_id, status);
}

/*============================================================================*/
/*=========================== Service Initialization =========================*/
/*============================================================================*/
static errcode_t sle_uuid_server_property_add(void)
{
    ssaps_property_info_t prop = {0};
    ssaps_desc_info_t desc = {0};
    uint8_t ntf_val[] = {0x01, 0x00};

    // 1. Notification Property (Data from Server to Client)
    prop.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    prop.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &prop.uuid);
    if (ssaps_add_property_sync(g_server_id, g_service_handle, &prop, &g_property_handle) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;

    desc.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    desc.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    desc.value = ntf_val;
    desc.value_len = sizeof(ntf_val);
    if (ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &desc) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;

    // 2. Command Property (Control from Client to Server)
    (void)memset_s(&prop, sizeof(ssaps_property_info_t), 0, sizeof(ssaps_property_info_t));
    prop.permissions = SSAP_PERMISSION_WRITE | SSAP_PERMISSION_ENCRYPTION_NEED;
    prop.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    sle_uuid_setu2(SLE_UUID_SERVER_CMD_CONTROL, &prop.uuid);
    if (ssaps_add_property_sync(g_server_id, g_service_handle, &prop, &g_command_property_handle) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;

    // 3. Status Property
    (void)memset_s(&prop, sizeof(ssaps_property_info_t), 0, sizeof(ssaps_property_info_t));
    prop.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    prop.operate_indication = SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    sle_uuid_setu2(SLE_UUID_SERVER_STATUS_REPORT, &prop.uuid);
    if (ssaps_add_property_sync(g_server_id, g_service_handle, &prop, &g_status_property_handle) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    if (ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_status_property_handle, &desc) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;

    // 4. NEW: Transmission Counter Property
    (void)memset_s(&prop, sizeof(ssaps_property_info_t), 0, sizeof(ssaps_property_info_t));
    prop.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_ENCRYPTION_NEED | SSAP_PERMISSION_AUTHENTICATION_NEED;
    prop.operate_indication = SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    sle_uuid_setu2(SLE_UUID_SERVER_TX_COUNT_REPORT, &prop.uuid);
    if (ssaps_add_property_sync(g_server_id, g_service_handle, &prop, &g_tx_count_property_handle) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    if (ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_tx_count_property_handle, &desc) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;

    return ERRCODE_SLE_SUCCESS;
}

static errcode_t sle_uart_server_add(void)
{
    sle_uuid_t app_uuid = {0};
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK) {
        // sample_at_log_print("%s App UUID memcpy failed\r\n", SLE_SERVER_LOG);
        return ERRCODE_SLE_FAIL;
    }
    (void)ssaps_register_server(&app_uuid, &g_server_id);

    sle_uuid_t service_uuid = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);
    if (ssaps_add_service_sync(g_server_id, &service_uuid, 1, &g_service_handle) != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id); return ERRCODE_SLE_FAIL;
    }
    if (sle_uuid_server_property_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id); return ERRCODE_SLE_FAIL;
    }
    if (ssaps_start_service(g_server_id, g_service_handle) != ERRCODE_SLE_SUCCESS) {
        // sample_at_log_print("%s Start service failed\r\n", SLE_SERVER_LOG);
        return ERRCODE_SLE_FAIL;
    }
    return ERRCODE_SLE_SUCCESS;
}

static void register_all_callbacks(void)
{
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.read_request_cb = ssaps_read_request_cbk;
    ssaps_cbk.write_request_cb = ssaps_write_request_cbk;
    ssaps_cbk.mtu_changed_cb = ssaps_mtu_changed_cbk;
    (void)ssaps_register_callbacks(&ssaps_cbk);
    sle_ssaps_set_info();
    
    sle_connection_callbacks_t conn_cbks = {0};
    conn_cbks.connect_state_changed_cb = sle_connect_state_changed_cbk;
    conn_cbks.pair_complete_cb = sle_pair_complete_cbk;
    // MODIFIED: Register handlers for connection parameter updates
    conn_cbks.connect_param_update_req_cb = sle_sample_update_req_cbk;
    conn_cbks.connect_param_update_cb = sle_sample_update_cbk;
    (void)sle_connection_register_callbacks(&conn_cbks);

    (void)sle_announce_seek_register_callbacks(&(sle_announce_seek_callbacks_t){0});
}


/*============================================================================*/
/*======================= Main Task & System Entry ===========================*/
/*============================================================================*/
static errcode_t sle_uart_server_send_report_by_handle(const uint8_t *data, uint16_t len)
{
    ssaps_ntf_ind_t param = {0};
    param.handle = g_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = (uint8_t*)data;
    param.value_len = len;
    return ssaps_notify_indicate(g_server_id, g_sle_conn_hdl, &param);
}

static errcode_t sle_uart_server_send_status_report(uint8_t status)
{
    ssaps_ntf_ind_t param = {0};
    param.handle = g_status_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = &status;
    param.value_len = 1;
    return ssaps_notify_indicate(g_server_id, g_sle_conn_hdl, &param);
}

// // NEW function to send transmission count
// static errcode_t sle_uart_server_send_transmission_count(uint8_t count)
// {
//     ssaps_ntf_ind_t param = {0};
//     param.handle = g_tx_count_property_handle;
//     param.type = SSAP_PROPERTY_TYPE_VALUE;
//     param.value = &count;
//     param.value_len = 1;
//     return ssaps_notify_indicate(g_server_id, g_sle_conn_hdl, &param);
// }

static void *sle_uart_server_task(void *arg)
{
    (void)arg;
    osal_task *sensor_task_handle = NULL;
    // sample_at_log_print("%s Main Server Task Started.\r\n", SLE_SERVER_LOG);
    print_memory_info("START");  // Check memory at start

    if (osal_msg_queue_create("drdy_queue", MSG_QUEUE_LEN, &g_drdy_msg_queue_id, 0, MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        // sample_at_log_print("%s FATAL: Failed to create DRDY message queue.\r\n", SLE_SERVER_LOG);
        return NULL;
    }
    if (osal_msg_queue_create("send_data_queue", MSG_QUEUE_LEN, &g_send_data_msg_queue_id, 0, MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        // sample_at_log_print("%s FATAL: Failed to create Send Data message queue.\r\n", SLE_SERVER_LOG);
        return NULL;
    }
    
    errcode_t wdt_ret = uapi_watchdog_init(WDT_SYSTEM_TIMEOUT_S);
    if (wdt_ret == ERRCODE_SUCC) {
        (void)uapi_watchdog_enable((wdt_mode_t)WDT_RESET_MODE);
        // sample_at_log_print("%s System-wide watchdog enabled with %ds timeout.\r\n", SLE_SERVER_LOG, WDT_SYSTEM_TIMEOUT_S);
    } else {
        sample_at_log_print("%s FATAL: FAILED to init watchdog!\r\n", SLE_SERVER_LOG);
    }

    register_all_callbacks();
    errcode_t ret = enable_sle();
    if (ret != ERRCODE_SLE_SUCCESS) {
        // sample_at_log_print("%s Failed to enable SLE stack.\r\n", SLE_SERVER_LOG);
        return NULL;
    }
    else {
        // sample_at_log_print("%s SLE stack enabled successfully.\r\n", SLE_SERVER_LOG);
    }

    print_memory_info("SLE_ENABLE");  // Check memory after SLE enable

    if (sle_uart_server_add() != ERRCODE_SLE_SUCCESS) {
        // sample_at_log_print("%s Failed to add UART server.\r\n", SLE_SERVER_LOG);
        return NULL;
    }
    
    sle_ssaps_set_info();
    sle_uart_server_adv_init();
    
    adxl355_init();
    
    freq = clocks_get_module_frequency(CLOCKS_CCRG_MODULE_MCU_CORE);
    osal_printk("%s Watchdog kicked. MCU Core Frequency: %u Hz\r\n", SLE_SERVER_LOG, freq);

    print_memory_info("INIT_DONE");  // Check memory after initialization

    osal_kthread_lock();
    sensor_task_handle = osal_kthread_create((osal_kthread_handler)sensor_sampling_task, NULL, "SensorSamplingTask", SENSOR_SAMPLING_TASK_STACK_SIZE);
    if (sensor_task_handle) {
        osal_kthread_set_priority(sensor_task_handle, SENSOR_SAMPLING_TASK_PRIO);
    }
    osal_kthread_unlock();
    
    osal_kthread_lock();
    sender_task_handle = osal_kthread_create((osal_kthread_handler)data_sending_task, NULL, "DataSendingTask", DATA_SENDING_TASK_STACK_SIZE);
    if (sender_task_handle) {
        osal_kthread_set_priority(sender_task_handle, DATA_SENDING_TASK_PRIO);
    }
    osal_kthread_unlock();

    while (1) {
        osal_msleep(10000);
        
        // REFACTORED: Conditional watchdog kicking logic
        if (!g_has_been_connected_once || g_is_sle_connected) {
            (void)uapi_watchdog_kick();
            
            print_memory_info("HeartBeat");  // Check memory
            // sample_at_log_print("%s Main task heartbeat. Watchdog kicked. Conn state: %d\r\n", SLE_SERVER_LOG, g_is_sle_connected);
        } else {
            sample_at_log_print("%s Main task heartbeat. DISCONNECTED. Watchdog NOT kicked.\r\n", SLE_SERVER_LOG);
        }
    }
    return NULL;
}

static void server_entry(void) {
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task,
                                      NULL, "ADXL355ServerTask", ADXL355_SERVER_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, ADXL355_SERVER_TASK_PRIO);
    }
    osal_kthread_unlock();
}

app_run(server_entry);
