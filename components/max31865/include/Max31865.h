#ifndef ESP32_MAX31865_H
#define ESP32_MAX31865_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>

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

#define MAX31865_FAULTSTATUS_RTDHIGH 7
#define MAX31865_FAULTSTATUS_RTDLOW 6
#define MAX31865_FAULTSTATUS_REFHIGH 5
#define MAX31865_FAULTSTATUS_REFLOW 4
#define MAX31865_FAULTSTATUS_RTDINLOW 3
#define MAX31865_FAULTSTATUS_VOLTAGE 2

enum class Max31865NWires : uint8_t { Three = 1, Two = 0, Four = 0 };
enum class Max31865FaultDetection : uint8_t {
  NoAction = 0b00,
  AutoDelay = 0b01,
  ManualDelayCycle1 = 0b10,
  ManualDelayCycle2 = 0b11
};
enum class Max31865Filter : uint8_t { Hz50 = 1, Hz60 = 0 };

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
  Max31865(int miso, int mosi, int sck, int cs);
  ~Max31865();

  esp_err_t begin(max31865_config_t config, max31865_rtd_config_t rtd);

  esp_err_t setConfig(max31865_config_t config);
  esp_err_t getConfig(max31865_config_t *config);
  esp_err_t clearFault();
  esp_err_t readFaultStatus(uint8_t *fault);

  esp_err_t getRTD(uint16_t *rtd);
  esp_err_t getTemperature(float *temperature);

 private:
  int miso;
  int mosi;
  int sck;
  int cs;
  max31865_config_t chipConfig;
  max31865_rtd_config_t rtdConfig;

  spi_host_device_t hostDevice;
  spi_device_handle_t deviceHandle;

  esp_err_t writeSPI(uint8_t addr, uint8_t *data, size_t size = 1);
  esp_err_t readSPI(uint8_t addr, uint8_t *result, size_t size = 1);
};

#endif  // ESP32_MAX31865_H
