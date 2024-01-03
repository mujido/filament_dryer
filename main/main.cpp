#include <esp_adc/adc_continuous.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ranges>
#include <numeric>
#include <array>

constexpr const char* TAG = "main";

constexpr uint32_t kAdcBufferSize = 1024;
constexpr uint32_t kAdcSampleRate = 20'000;
constexpr uint32_t kAdcSamplesToRead = 100;
constexpr uint32_t kAdcSampleReadSize = kAdcSamplesToRead * SOC_ADC_DIGI_RESULT_BYTES;
constexpr auto kAdcBitWidth = ADC_BITWIDTH_10;
constexpr auto kAdcUnit = ADC_UNIT_1;
constexpr auto kAdcChannel = ADC_CHANNEL_6;

static_assert(kAdcSampleRate >= SOC_ADC_SAMPLE_FREQ_THRES_LOW && kAdcSampleRate <= SOC_ADC_SAMPLE_FREQ_THRES_HIGH, "ADC sample rate out of range");

static bool continuous_adc_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data)
{
    auto main_task = *reinterpret_cast<TaskHandle_t*>(user_data);

    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(main_task, &mustYield);
    return mustYield == pdTRUE;
}

static adc_continuous_handle_t continuous_adc_init()
{
    adc_continuous_handle_t handle = nullptr;

    adc_continuous_handle_cfg_t adc_config{};
    adc_config.max_store_buf_size = kAdcBufferSize;
    adc_config.conv_frame_size = kAdcSampleReadSize;
    adc_config.flags.flush_pool = 1;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = kAdcChannel;
    adc_pattern[0].unit = kAdcUnit;
    adc_pattern[0].bit_width = kAdcBitWidth;

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = kAdcSampleRate,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    return handle;
}

extern "C" void app_main()
{
    auto adc_handle = continuous_adc_init();
    assert(adc_handle != nullptr);

    auto main_task = xTaskGetCurrentTaskHandle();

    adc_continuous_evt_cbs_t adc_cbs = {
        .on_conv_done = continuous_adc_done_callback,
        .on_pool_ovf = nullptr
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &adc_cbs, &main_task));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while(1) {
        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            uint32_t ret_bytes = 0;
            static std::array<adc_digi_output_data_t, kAdcSamplesToRead> readings;

            auto ret = adc_continuous_read(adc_handle, reinterpret_cast<uint8_t*>(readings.data()), sizeof(readings), &ret_bytes, 0);
            if (ret == ESP_OK) {
                auto reading_count = ret_bytes / sizeof(readings[0]);
                uint32_t avg = std::accumulate(readings.begin(), readings.begin() + reading_count, uint32_t{}, [](uint32_t accum, const adc_digi_output_data_t& reading)
                {
                    return accum + reading.type1.data;
                });

                avg /= reading_count;

                // Correct for non-lineararity in ESP32 ADC.
                const float avg2 = avg * avg;
                const float avg3 = avg2 * avg;
                const float adc_corr = 40.4597f + 0.976323f*avg + 0.000163748f*avg2 - 1.76614e-7f*avg3;

                // Calculate temperature based off calibration curve
                const float adc_corr2 = adc_corr * adc_corr;
                const float temp = 129.85f - 0.150499*adc_corr + 0.0000343308f*adc_corr2;

                const float voltage = adc_corr * 3.3f / (1 << kAdcBitWidth);

                ESP_LOGI(TAG, "Avg reading: %lu corrected %lu (%.1f) [%.4fV]", avg, (uint32_t)adc_corr, temp, voltage);

                vTaskDelay(1000 / portTICK_PERIOD_MS);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
}