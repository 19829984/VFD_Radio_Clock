#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define ADC_READ_LEN 128
#define _ADC_UNIT_STR(unit)         #unit
#define ADC_UNIT_STR(unit)          _ADC_UNIT_STR(unit)

static void init_adc_1(adc_channel_t channel, adc_continuous_handle_t *out_handle) {
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

    ESP_LOGI("Setup", "adc_pattern.atten is :%"PRIx8, adc_pattern[0].atten);
    ESP_LOGI("Setup", "adc_pattern.channel is :%"PRIx8, adc_pattern[0].channel);
    ESP_LOGI("Setup", "adc_pattern.unit is :%"PRIx8, adc_pattern[0].unit);
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

// static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
// {
//     uint32_t conversion_size = edata->size;
//     uint8_t *conversion_buffer = edata->conv_frame_buffer;
// }

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[ADC_READ_LEN] = {0};
    memset(result, 0xcc, ADC_READ_LEN);
    
    adc_continuous_handle_t handle = NULL;
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
    init_adc_1(ADC_CHANNEL_0, &handle);
    
    // adc_continuous_evt_cbs_t cbs = {
    //     .on_conv_done = s_conv_done_cb,
    // };
    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    uint32_t sum = 0;
    uint16_t sum_index = 0;
    const uint16_t sum_limit = 500;
    int avg_raw_data = 0;
    int mv_data = 0;
    while (1) {
        ret = adc_continuous_read(handle, result, ADC_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            // ESP_LOGI("Read", "ret_num is :%"PRIu32"", ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type2.channel;
                uint32_t data = p->type2.data;
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                    if (sum_index != sum_limit) {
                        sum += data;
                        sum_index++;
                    } else {
                        avg_raw_data = sum / sum_limit;
                        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, avg_raw_data, &mv_data));
                        ESP_LOGI("READ", "Unit: %s, Channel: %"PRIu32", Value: %.3fV", ADC_UNIT_STR(ADC_UNIT_1), chan_num, mv_data/1000.0);
                        sum = 0;
                        sum_index = 0;
                    }
                    // ESP_LOGI("READ", "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, ADC_UNIT_STR(ADC_UNIT_1), chan_num, data);
                } else {
                    ESP_LOGW("READ", "Invalid data [%s_%"PRIu32"_%"PRIx32"]", ADC_UNIT_STR(ADC_UNIT_1), chan_num, data);
                }
            }
        } else {
            ESP_LOGE("Read", "adc_continuous_read failed, ret:%d", ret);
        }
        vTaskDelay(1);
    }
    
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}