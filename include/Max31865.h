#ifndef ESP32_MAX31865_H
#define ESP32_MAX31865_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define MAX31865_CONFIG_REG 0x00
#define MAX31865_RTD_REG 0x01
#define MAX31865_HIGH_FAULT_REG 0x03
#define MAX31865_LOW_FAULT_REG 0x05
#define MAX31865_FAULT_STATUS_REG 0x07

#define MAX31865_REG_WRITE_OFFSET 0x80

#define MAX31865_CONFIG_VBIAS_BIT 7
#define MAX31865_CONFIG_CONVERSIONMODE_BIT 6
#define MAX31865_CONFIG_1SHOT_BIT 5
#define MAX31865_CONFIG_NWIRES_BIT 4
#define MAX31865_CONFIG_FAULTDETECTION_BIT 2
#define MAX31865_CONFIG_FAULTSTATUS_BIT 1
#define MAX31865_CONFIG_MAINSFILTER_BIT 0

enum class Max31865NWires : uint8_t { Three = 1, Two = 0, Four = 0 };
enum class Max31865FaultDetection : uint8_t {
  NoAction = 0b00,
  AutoDelay = 0b01,
  ManualDelayCycle1 = 0b10,
  ManualDelayCycle2 = 0b11
};
enum class Max31865Filter : uint8_t { Hz50 = 1, Hz60 = 0 };
enum class Max31865Error : uint8_t {
  NoError = 0,
  Voltage = 2,
  RTDInLow,
  RefLow,
  RefHigh,
  RTDLow,
  RTDHigh
};

struct max31865_config_t {
  bool vbias;
  bool autoConversion;
  Max31865NWires nWires;
  Max31865FaultDetection faultDetection;
  Max31865Filter filter;
};

struct max31865_rtd_config_t {
  float ref;
  float nominal;
};

class Max31865 {
 public:
  static float RTDtoTemperature(uint16_t rtd, max31865_rtd_config_t rtdConfig);
  static uint16_t temperatureToRTD(float temperature,
                                   max31865_rtd_config_t rtdConfig);
  static const char *errorToString(Max31865Error error);

  Max31865(int miso, int mosi, int sck, int cs, int drdy = -1,
           spi_host_device_t host = HSPI_HOST);
  ~Max31865();

  esp_err_t begin(max31865_config_t config);
  esp_err_t setConfig(max31865_config_t config);
  esp_err_t getConfig(max31865_config_t *config);
  esp_err_t setRTDThresholds(uint16_t min, uint16_t max);
  esp_err_t clearFault();
  esp_err_t readFaultStatus(Max31865Error *fault);
  esp_err_t getRTD(uint16_t *rtd, Max31865Error *fault = nullptr);

 private:
  static void drdyInterruptHandler(void *arg);

  int miso;
  int mosi;
  int sck;
  int cs;
  int drdy;
  spi_host_device_t hostDevice;
  max31865_config_t chipConfig;
  spi_device_handle_t deviceHandle;
  SemaphoreHandle_t drdySemaphore;

  esp_err_t writeSPI(uint8_t addr, uint8_t *data, size_t size = 1);
  esp_err_t readSPI(uint8_t addr, uint8_t *result, size_t size = 1);
};

#endif  // ESP32_MAX31865_H
