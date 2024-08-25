/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_check.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue;

// Wakeup
esp_err_t register_gpio_wakeup(void);
void wait_gpio_inactive(void);
#define GPIO_WAKEUP_NUM 4
#define GPIO_WAKEUP_LEVEL 1
int64_t start_time = 0;

static uint8_t peer_addr[6] = {0xf0, 0xf5, 0xbd, 0xfc, 0x00, 0xc8};

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels

#define ADC1_CHAN0 ADC_CHANNEL_6
#define ADC1_CHAN1 ADC_CHANNEL_7

static int adc_raw1;
static int adc_raw2;

typedef enum
{
    POS_UP = 1,
    POS_DOWN,
    POS_LEFT,
    POS_RIGHT,
    POS_DEFAULT_X,
    POS_DEFAULT_Y

} pos_joystick_t;

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;

    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case EXAMPLE_ESPNOW_SEND_CB:
        {
            // example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

            // ESP_LOGI(TAG, "Send data to " MACSTR ", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
            break;
        }
        case EXAMPLE_ESPNOW_RECV_CB:
        {
            break;
        }
        }
    }
}

static esp_err_t espnow_init(void)
{

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add peer address information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    xTaskCreate(espnow_task, "espnow_task", 2048, NULL, 4, NULL);

    return ESP_OK;
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    espnow_init();

    //-------------ADC Init---------------//
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC1_CHAN1, &config));

    /* Enable wakeup from light sleep by gpio */
    register_gpio_wakeup();

    uint8_t data[2] = {0};
    size_t len = sizeof(data);

    start_time = esp_timer_get_time();

    while (1)
    {
        adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc_raw1);
        adc_oneshot_read(adc_handle, ADC1_CHAN1, &adc_raw2);

        data[0] = (uint8_t)adc_raw2;
        data[1] = (uint8_t)adc_raw1;

        // ESP_LOGI(TAG, "x = %d", data[0]);
        // ESP_LOGI(TAG, "y = %d", data[1]);
        // ESP_LOGI(TAG, "GPIO15 = %d", gpio_get_level(GPIO_WAKEUP_NUM));

        esp_now_send(peer_addr, data, len);

        vTaskDelay(pdMS_TO_TICKS(100));

        int64_t current_time = esp_timer_get_time();

        if ((data[0] != 0 && data[0] != 255 && data[1] != 0 && data[1] != 255 && (current_time - start_time) >= 100000000) || (current_time - start_time) >= 300000000)
        {

            ESP_LOGI(TAG, "Entering light sleep mode...");

            esp_now_deinit();
            esp_wifi_stop();

            // Reset time
            start_time = esp_timer_get_time();

            /* Enter sleep mode */
            esp_light_sleep_start();

            wait_gpio_inactive();

            esp_wifi_start();
            espnow_init();
        }
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    esp_now_deinit();
}

esp_err_t register_gpio_wakeup(void)
{
    /* Initialize GPIO */
    gpio_config_t config = {
        .pin_bit_mask = BIT64(GPIO_WAKEUP_NUM),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = false,
        .pull_up_en = true,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Initialize GPIO%d failed", GPIO_WAKEUP_NUM);

    /* Enable wake up from GPIO */
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(GPIO_WAKEUP_NUM, GPIO_INTR_LOW_LEVEL),
                        TAG, "Enable gpio wakeup failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "Configure gpio as wakeup source failed");

    /* Make sure the GPIO is inactive and it won't trigger wakeup immediately */
    wait_gpio_inactive();
    ESP_LOGI(TAG, "gpio wakeup source is ready");

    return ESP_OK;
}

void wait_gpio_inactive(void)
{
    printf("Waiting for GPIO%d to go high...\n", GPIO_WAKEUP_NUM);
    while (gpio_get_level(GPIO_WAKEUP_NUM) == GPIO_WAKEUP_LEVEL)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
