# ESP32-MAX31865
A driver for the MAX31865 RTD-to-Digital converter, written using/for the [esp-idf](<https://github.com/espressif/esp-idf>) framework. Supports all documented features of the MAX31865 including fault detection and the DRDY output. 

### Example

See [examples/simple/main/main.cpp](examples/simple/main/main.cpp).

### Reference

#### Constructor

##### `Max31856(int miso, int mosi, int sck, int cs, int drdy = -1, spi_host_device_t host = HSPI_HOST)`

###### Parameters

- `int miso, int mosi, int sck, int cs` - GPIO numbers for SPI communication to the chip
- `int drdy` - GPIO number for the DRDY output of the MAX31865. Not used if equal to `-1`. If set, RTD reads will block until DRDY is low.
- `spi_host_device_t host` - ESP32 SPI [host device](<https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html#_CPPv417spi_host_device_t>), useful if the application needs to communicate with other SPI devices simultaneously.



#### Static methods

##### `float RTDtoTemperature(uint16_t rtd, max31865_rtd_config_t rtdConfig)`

###### Parameters

- `uint16_t rtd` - 15-bit RTD resistance value read from the MAX31865
-  `max31865_rtd_config_t rtdConfig` - configuration relating to the RTD itself

###### Returns

- `float` - the temperature value in degrees Celsius represented by the RTD resistance with the given configuration. Uses the Callendar-van Dusen equation with constants taken from [here](https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf).

  

##### `uint16_t temperatureToRTD(float temperature, max31865_rtd_config_t rtdConfig)`

###### Parameters

- `float temperature` - a temperature in degrees Celsius.
-  `max31865_rtd_config_t rtdConfig` - configuration relating to the RTD itself

###### Returns

- `uint16_t` - 15-bit RTD resistance value as would be represented in the MAX31865 registers.



##### `const char *errorToString(Max31865Error error)`

###### Parameters

- `Max31865Error error` - an error type read from the MAX31865 fault register.

###### Returns

- `const char*` - a string describing the error type.



#### Instance methods

##### `esp_err_t begin(max31865_config_t config)`

Initialises the communication with the MAX31865, and configures it. Must be called before any other instance methods!

###### Parameters

- `max31865_config_t config` - configuration for the MAX31865

###### Returns

- `esp_err_t` - any error encountered when communicating with the chip.



##### `esp_err_t setConfig(max31865_config_t config)`

Sets the configuration for an already initialized instance.

###### Parameters

- `max31865_config_t config` - configuration for the MAX31865



##### `esp_err_t getConfig(max31865_config_t *config)`

Gets the current configuration from a MAX31865.

###### Parameters

- `max31865_config_t *config` - pointer to the configuration struct to be populated with the current state.



##### `esp_err_t setRTDThresholds(uint16_t min, uint16_t max)`

Sets the threshold RTD resistances (_not_ temperatures) below/above which the MAX31865 will declare a fault. Use with `MAX31865::temperatureToRTD` for setting min/max temperatures.

###### Parameters

- `uint16_t min` - minimum RTD resistance value (15-bit)
- `uint16_t max` - maximum RTD resistance value (15-bit)



##### `esp_err_t clearFault()`

Clears the fault status bit in the config register.



##### `esp_err_t readFaultStatus(Max31865Error *fault);`

Reads the specific fault type out of the fault status register.

###### Parameters

- `Max31865Error *fault` - pointer to the fault type variable to be filled (output param).



##### `esp_err_t getRTD(uint16_t *rtd, Max31865Error *fault = nullptr)`

Gets the current RTD resistance (15-bit value) from the MAX31865 and optionally populates a fault variable. Fails with `ESP_ERR_INVALID_RESPONSE` if a fault is detected. Does the following in the process of reading:

- Enables a 1-shot read and/or Vbias if they are not available, and blocks for 65ms and 10ms, respectively, after doing so in order to have valid reads.
- If a pin for DRDY was given, blocks until receiving a negative edge interrupt on this pin.
- Reads the fault bit, and if equal to 1 reads the full fault status using `readingFaultStatus`
- Restores Vbias to 0 if it was changed for the read.

###### Parameters

- `uint16_t *rtd` - pointer to the RTD resistance value (15-bit) to populate.
- `Max31865Error *fault` - optionally, an error type to populate if a fault is detected.



#### Types

##### `struct max31865_config_t`

###### Members

- `bool vbias` - Enables Vbias if true
- `bool autoConversion` - Enables automatic conversion if true (otherwise 1-shot)
- `Max31865NWires nWires` - 2, 3 or 4-wire RTD configuration
- `Max31865FaultDetection faultDetection` - which fault detection cycle mode to enable
- `Max31865Filter filter` - mains sinc filter frequenct



##### `struct max31865_rtd_config_t`

###### Members

- `float ref` - reference resistor value (Ohms)
- `float nominal` - nominal RTD resistance at 0 degrees C (eg PT100 or PT1000)



##### `enum class Max31865NWires`

RTD wiring configuration

- `Three`
- `Two` - default
- `Four ` - default



##### `enum class Max31865FaultDetection`

Fault detection cycle

- `NoAction` - default
- `AutoDelay` - Fault detection with automatic delay
- `ManualDelayCycle1` - Run fault detection with manual delay (cycle 1)
- `ManualDelayCycle2` - Finish fault detection with manual delay
  (cycle 2)



##### `enum class Max31865Filter`

Mains frequency filter

- `Hz50` - filter 50Hz mains
- `Hz60` - filter 60Hz mains (default)



##### `enum class Max31865Error`

Fault status register error type

- `NoError` - default
- `Voltage` - Overvoltage or undervoltage fault
- `RTDInLow` - RTDIN- < 0.85*VBIAS (FORCE- open)
- `RefLow` - REFIN- < 0.85*VBIAS (FORCE- open)
- `RefHigh` - REFIN- > 0.85*VBIAS
- `RTDLow` - RTD below low threshold
- `RTDHigh` - RTD above high threshold

