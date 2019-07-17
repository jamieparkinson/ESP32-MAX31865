#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Max31865.h"

extern "C" void app_main() {
  auto tempSensor = Max31865(12, 13, 14, 15);
  max31865_config_t tempConfig = {};
  tempConfig.autoConversion = true;
  tempConfig.vbias = true;
  tempConfig.filter = Max31865Filter::Hz50;
  tempConfig.nWires = Max31865NWires::Three;
  max31865_rtd_config_t rtdConfig = {};
  rtdConfig.nominal = 100.0f;
  rtdConfig.ref = 430.0f;
  ESP_ERROR_CHECK(tempSensor.begin(tempConfig, rtdConfig));

  while (true) {
    float temp;
    ESP_ERROR_CHECK(tempSensor.getTemperature(&temp));
    ESP_LOGI("Temperature", "%.2f C", temp);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
