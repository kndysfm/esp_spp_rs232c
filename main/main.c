/*
 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "esp_system.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)  //!< Transmitted Data [out]
#define RXD_PIN (GPIO_NUM_5)  //!< Received Data [in]
#define DTR_PIN (GPIO_NUM_16) //!< Data Terminal Ready [out]
#define DSR_PIN (GPIO_NUM_17) //!< Data Set Ready [in]
#define RTS_PIN (GPIO_NUM_2)  //!< Request To Send [out]
#define CTS_PIN (GPIO_NUM_15) //!< Clear To Send [in]

static void uart_init()
{
    const uart_config_t uart_config = { .baud_rate = 57600, .data_bits =
            UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
            UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
    UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static uint32_t _spp_handle = 0;

static void rx_task()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE,
                1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            if (_spp_handle) esp_spp_write(_spp_handle, rxBytes, data);
        }
    }
    free(data);
}

static void myuart_setup()
{
    uart_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}

#define SPP_TAG "SPP_RS232C"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXCAMPLE_DEVICE_NAME "ESP_SPP_RS232C"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint16_t _port_handle = 0;
/// from $IDF_PATH/components/bt/bluedroid/stack/include/stack
#define PORT_DTRDSR_ON          0x01
#define PORT_CTSRTS_ON          0x02
extern int PORT_GetModemStatus(uint16_t handle, uint8_t *p_signal);
#define PORT_SET_DTRDSR         0x01
#define PORT_CLR_DTRDSR         0x02
#define PORT_SET_CTSRTS         0x03
#define PORT_CLR_CTSRTS         0x04
extern int PORT_Control (uint16_t handle, uint8_t signal);

static void port_task()
{
    static const char *PORT_TASK_TAG = "PORT_TASK";
    while (1) {
        // initialize GPIO pins for control signals
        gpio_pad_select_gpio(RTS_PIN);
        gpio_set_direction(RTS_PIN, GPIO_MODE_OUTPUT);
        gpio_pad_select_gpio(DTR_PIN);
        gpio_set_direction(DTR_PIN, GPIO_MODE_OUTPUT);
        gpio_pad_select_gpio(CTS_PIN);
        gpio_set_direction(CTS_PIN, GPIO_MODE_INPUT);
        gpio_pad_select_gpio(DSR_PIN);
        gpio_set_direction(DSR_PIN, GPIO_MODE_INPUT);
        if (_port_handle) {
            static uint8_t sig_prev_in = 0x80;
            uint8_t sig;
            if (!PORT_GetModemStatus(_port_handle, &sig)) {
                if (sig_prev_in != sig) {
                    ESP_LOGI(PORT_TASK_TAG, "DTR=%d, RTS=%d",
                            !!(sig & PORT_DTRDSR_ON), !!(sig & PORT_CTSRTS_ON));
                    sig_prev_in = sig;
                }
                gpio_set_level(DTR_PIN, !(sig & PORT_DTRDSR_ON)); //DTR --> output L
                gpio_set_level(RTS_PIN, !(sig & PORT_CTSRTS_ON)); //RTS --> output L
            }

            PORT_Control(_port_handle, gpio_get_level(CTS_PIN)? PORT_CLR_CTSRTS: PORT_SET_CTSRTS); // ineffective ?
            PORT_Control(_port_handle, gpio_get_level(DSR_PIN)? PORT_CLR_DTRDSR: PORT_SET_DTRDSR);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
            _port_handle = 0;
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                    param->data_ind.len, param->data_ind.handle);
            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
            uart_write_bytes(UART_NUM_1, (char *) param->data_ind.data,
                    (size_t) param->data_ind.len);
            _spp_handle = param->data_ind.handle;
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d",
                    param->write.len, param->write.cong);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
            _port_handle = 0;
            for (uint16_t h = 1; h < 30; h++) { // MAX_RFC_PORTS <= 30
                uint8_t sig;
                int res = PORT_GetModemStatus(h, &sig);
                if (res == 0) { //PORT_SUCCESS
                    _port_handle = h;
                    ESP_LOGI(SPP_TAG, "Opened port:%d", h);
                    break;
                } else {
                    ESP_LOGI(SPP_TAG, "Not opened port:%d res:%d", h, res);
                }
            }
            break;
        default:
            ESP_LOGI(SPP_TAG, "event: %d", event);
            break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_BT_SSP_ENABLE
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(SPP_TAG, "authentication success: %s",
                        param->auth_cmpl.device_name);
                esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
                        param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(SPP_TAG,
                    "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d",
                    param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d",
                    param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
#endif ///CONFIG_BT_SSP_ENABLE
        default: {
            ESP_LOGI(SPP_TAG, "event: %d", event);
            break;
        }
    }
    return;
}

void app_main()
{
    myuart_setup();
    xTaskCreate(port_task, "rfcomm_port_task", 1024 * 2, NULL,
    configMAX_PRIORITIES - 1, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
    ;
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__,
                esp_err_to_name(ret));
        return;
    }

#ifdef CONFIG_BT_SSP_ENABLE
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif ///CONFIG_BT_SSP_ENABLE

}

