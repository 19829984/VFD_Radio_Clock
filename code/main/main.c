#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/dedic_gpio.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "freertos/event_groups.h"
#include "vfd_digits.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define DEBUG

// WIFI
#define WIFI_SSID "EECS_Labs"
#define WIFI_PASS ""

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 10)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WIFI Handler", "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("WIFI Handler", "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI Handler", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ADC
#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define ADC_READ_LEN 128
#define _ADC_UNIT_STR(unit) #unit
#define ADC_UNIT_STR(unit) _ADC_UNIT_STR(unit)

// GPIO
#define V27_EN_PIN 21
#define VFD0_EN_PIN 13
#define VFD1_EN_PIN 14
#define VFD2_EN_PIN 17
#define VFD3_EN_PIN 18
#define SHIFT_REG_OE_PIN 48
#define GPIO_OUTPUT_PIN_SEL ((1ULL << V27_EN_PIN) | (1ULL << SHIFT_REG_OE_PIN))

// SPI
#define SPI_HOST SPI2_HOST
#define SPI2_CS_PIN 10
#define SPI2_DATA_PIN 11
#define SPI2_CLK_PIN 12

// LEDC
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO VFD0_EN_PIN
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO VFD1_EN_PIN
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1
#define LEDC_HS_CH2_GPIO VFD2_EN_PIN
#define LEDC_HS_CH2_CHANNEL LEDC_CHANNEL_2
#define LEDC_HS_CH3_GPIO VFD3_EN_PIN
#define LEDC_HS_CH3_CHANNEL LEDC_CHANNEL_3
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (2048)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (100)            // Frequency in Hertz. Set frequency at 100 Hz

uint8_t digit_index = 0;
uint8_t vfd_digits_io[4] = {VFD0_EN_PIN, VFD2_EN_PIN, VFD3_EN_PIN, VFD1_EN_PIN};
uint8_t digits_to_display[4] = {1, 2, 3, 4};
uint8_t decimal_point = 0b00000000;
spi_device_handle_t spi_handle = NULL;
dedic_gpio_bundle_handle_t bundleA = NULL;

spi_transaction_t vfd_transaction = {
    .flags = SPI_TRANS_USE_TXDATA,
    .length = 8,
    // .tx_buffer = vfd_digits,
};

inline bool is_decimal(uint8_t digit)
{
    return (decimal_point & (0b00000001 << digit)) > 0;
}

inline void set_decimal(uint8_t digit)
{
    decimal_point |= 0b00000001 << digit;
}

inline void set_decimal_mask(uint8_t mask)
{
    decimal_point = mask;
}

// TIMER
static bool change_digit(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    digit_index = (digit_index + 1) & 0b11;
    vfd_transaction.tx_data[0] = vfd_digits[digits_to_display[digit_index] + (is_decimal(digit_index)  << 4)];
    dedic_gpio_bundle_write(bundleA, 0b1111, 0b0001 << digit_index);
    spi_device_queue_trans(spi_handle, &vfd_transaction, 1);
    return true;
}

static void init_adc_1(adc_channel_t channel, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 1;
    adc_pattern[0].atten = ADC_ATTENUATION;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI("Setup", "adc_pattern.atten is :%" PRIx8, adc_pattern[0].atten);
    ESP_LOGI("Setup", "adc_pattern.channel is :%" PRIx8, adc_pattern[0].channel);
    ESP_LOGI("Setup", "adc_pattern.unit is :%" PRIx8, adc_pattern[0].unit);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_HUNT_AND_PECK,
            .sae_h2e_identifier = "",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI INIT", "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("WIFI INIT", "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI("WIFI INIT", "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGE("WIFI INIT", "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_READ_LEN] = {0};
    memset(result, 0xcc, ADC_READ_LEN);

    adc_continuous_handle_t handle = NULL;
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
    init_adc_1(ADC_CHANNEL_0, &handle);

    // adc_continuous_evt_cbs_t cbs = {
    //     .on_conv_done = s_conv_done_cb,
    // };
    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    // GPIO Setup
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << V27_EN_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << SHIFT_REG_OE_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // SPI Setup
    esp_err_t err = ESP_OK;
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num = -1,
        .mosi_io_num = SPI2_DATA_PIN,
        .sclk_io_num = SPI2_CLK_PIN,
    };
    ret = spi_bus_initialize(SPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t spi_interface_cfg = {
        .clock_speed_hz = (1 * 1000 * 1000), // 1 MHz
        .command_bits = 0,
        .address_bits = 0,
        .queue_size = 1,
        .mode = 0,
        .spics_io_num = SPI2_CS_PIN,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST};
    err = spi_bus_add_device(SPI_HOST, &spi_interface_cfg, &spi_handle);
    if (err)
    {
        ESP_LOGE("SPI", "SPI Initialization failed!");
    }
    uint8_t tx_data_test = 0; // 0b01010101
    uint8_t vfd_digit_index = 0;
    spi_transaction_t test_transaction = {
        // .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
        .tx_buffer = &tx_data_test,
    };
    // spi_device_queue_trans(spi_handle, &test_transaction, 0);

    // LEDC PWM
    // ledc_timer_config_t ledc_timer = {
    //     .duty_resolution = LEDC_DUTY_RES, // resolution of PWM duty
    //     .freq_hz = LEDC_FREQUENCY,                      // frequency of PWM signal
    //     .speed_mode = LEDC_MODE,           // timer mode
    //     .timer_num = LEDC_TIMER,            // timer index
    //     .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    // };
    // ledc_timer_config(&ledc_timer);

    // ledc_channel_config_t ledc_channel[4] = {
    //     {
    //         .channel    = LEDC_HS_CH0_CHANNEL,
    //         .duty       = LEDC_DUTY,
    //         .gpio_num   = LEDC_HS_CH0_GPIO,
    //         .speed_mode = LEDC_MODE,
    //         .hpoint     = 0,
    //         .timer_sel  = LEDC_TIMER,
    //         .flags.output_invert = 0
    //     },
    //     {
    //         .channel    = LEDC_HS_CH1_CHANNEL,
    //         .duty       = LEDC_DUTY,
    //         .gpio_num   = LEDC_HS_CH1_GPIO,
    //         .speed_mode = LEDC_MODE,
    //         .hpoint     = 2048,
    //         .timer_sel  = LEDC_TIMER,
    //         .flags.output_invert = 0
    //     },
    //     {
    //         .channel    = LEDC_HS_CH2_CHANNEL,
    //         .duty       = LEDC_DUTY,
    //         .gpio_num   = LEDC_HS_CH2_GPIO,
    //         .speed_mode = LEDC_MODE,
    //         .hpoint     = 4096,
    //         .timer_sel  = LEDC_TIMER,
    //         .flags.output_invert = 0
    //     },
    //     {
    //         .channel    = LEDC_HS_CH3_CHANNEL,
    //         .duty       = LEDC_DUTY,
    //         .gpio_num   = LEDC_HS_CH3_GPIO,
    //         .speed_mode = LEDC_MODE,
    //         .hpoint     = 6144,
    //         .timer_sel  = LEDC_TIMER,
    //         .flags.output_invert = 0
    //     },
    // };

    // for (int ch = 0; ch < 4; ch++){
    //     ledc_channel_config(&ledc_channel[ch]);
    // }

    // Dedicated GPIO
    const int bundleA_gpios[] = {VFD0_EN_PIN, VFD2_EN_PIN, VFD3_EN_PIN, VFD1_EN_PIN};
    gpio_config_t dedicated_io_conf = {
        .mode = GPIO_MODE_OUTPUT,
    };
    for (int i = 0; i < sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]); i++)
    {
        dedicated_io_conf.pin_bit_mask = 1ULL << bundleA_gpios[i];
        gpio_config(&dedicated_io_conf);
    }
    dedic_gpio_bundle_config_t bundleA_config = {
        .gpio_array = bundleA_gpios,
        .array_size = sizeof(bundleA_gpios) / sizeof(bundleA_gpios[0]),
        .flags = {
            .out_en = 1,
        },
    };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleA));

    // Timer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1 * 1000, // 4000 us or 4 ms per alarm, 250 Hz
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = change_digit, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &digit_index));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    uint32_t sum = 0;
    uint16_t sum_index = 0;
    const uint16_t sum_limit = 500;
    int avg_raw_data = 0;
    int mv_data = 0;

    gpio_set_level(V27_EN_PIN, 1);
    gpio_pulldown_en(SHIFT_REG_OE_PIN);
    gpio_set_level(SHIFT_REG_OE_PIN, 0);
    // gpio_set_level(VFD0_EN_PIN, 0);
    // gpio_set_level(VFD1_EN_PIN, 1);
    // gpio_set_level(VFD2_EN_PIN, 0);
    // gpio_set_level(VFD3_EN_PIN, 0);
    // gpio_dump_io_configuration();
    tx_data_test = 255;

#ifdef DEBUG
    uint16_t digit = 0;
    // set_decimal_mask(0x0F);
    set_decimal(1);
    while (1){
        digits_to_display[0] = (digit >> 12);
        digits_to_display[1] = ((digit >> 8) & 0xF);
        digits_to_display[2] = (digit >> 4) & 0xF;
        digits_to_display[3] = digit & 0xF;
    
        // if (++digit == NUM_VFD_DIGITS)
        // if (digit == 16)
            // digit = 0;
        // else
        //     digit = 16;
        // ESP_LOGI("DEBUG", "Digit: %d", digit);
        uint64_t count = 0;
        gptimer_get_raw_count(gptimer, &count);
        // ESP_LOGI("DEBUG", "Alarm count: %llu", count);
        // ESP_LOGI("DEBUG", "Vfd Transaction: %p", vfd_transaction.tx_buffer);
        // ESP_LOGI("DEBUG", "Vfd digit_index: %d", digit_index);
        digit++;
        vTaskDelay(8);
    }
#else

    wifi_init_sta();
    time_t now;
    // char strftime_buf[64];
    struct tm timeinfo;

    setenv("TZ", "UTC-5", 1);
    tzset();

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
    {
        ESP_LOGE("NETIF SNTP", "Failed to update system time within 10s timeout");
    }
    while (1)
    {
        time(&now);
        localtime_r(&now, &timeinfo);
        // ESP_LOGI("TIME MIN", "min: %d", timeinfo.tm_min);
        // ESP_LOGI("TIME SEC", "sec: %d", timeinfo.tm_sec);
        digits_to_display[0] = timeinfo.tm_min / 10;
        digits_to_display[1] = timeinfo.tm_min % 10;
        digits_to_display[2] = timeinfo.tm_sec / 10;
        digits_to_display[3] = timeinfo.tm_sec % 10;
        // strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        // ESP_LOGI("TIME", "The current date/time in Boston is: %s", strftime_buf);
        // gettimeofday
        // ret = adc_continuous_read(handle, result, ADC_READ_LEN, &ret_num, 0);
        // tx_data_test = 0b00000001 << vfd_digit_index;
        // tx_data_test = vfd_digits[vfd_digit_index];
        // spi_device_queue_trans(spi_handle, &test_transaction, 0);
        // ESP_LOGI("Shift Register", "Sent data %d", tx_data_test);
        // vfd_digit_index = (vfd_digit_index + 1) % NUM_VFD_DIGITS;
        // printf("VFD Digit Index: %d", digit_index);
        // if (ret == ESP_OK) {
        //     // ESP_LOGI("Read", "ret_num is :%"PRIu32"", ret_num);
        //     for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
        //         adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
        //         uint32_t chan_num = p->type2.channel;
        //         uint32_t data = p->type2.data;
        //         /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
        //         if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
        //             if (sum_index != sum_limit) {
        //                 sum += data;
        //                 sum_index++;
        //             } else {
        //                 avg_raw_data = sum / sum_limit;
        //                 ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, avg_raw_data, &mv_data));
        //                 ESP_LOGI("READ", "Unit: %s, Channel: %"PRIu32", Value: %.3fV", ADC_UNIT_STR(ADC_UNIT_1), chan_num, mv_data/1000.0);
        //                 sum = 0;
        //                 sum_index = 0;
        //             }
        //             // ESP_LOGI("READ", "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, ADC_UNIT_STR(ADC_UNIT_1), chan_num, data);
        //         } else {
        //             ESP_LOGW("READ", "Invalid data [%s_%"PRIu32"_%"PRIx32"]", ADC_UNIT_STR(ADC_UNIT_1), chan_num, data);
        //         }
        //     }
        // } else {
        //     ESP_LOGE("Read", "adc_continuous_read failed, ret:%d", ret);
        // }
        vTaskDelay(100);
    }
#endif
   vTaskDelay(100);
    }

