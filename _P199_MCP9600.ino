#ifdef USES_P199
//###########################################################################################
//#################### Plugin 199 MCP9600 I2C thermocouple interface  #######################
//###########################################################################################

//#include <math.h>
#include <map>

#define PLUGIN_199
#define PLUGIN_ID_199       199 
#define PLUGIN_NAME_199       "Temperature - MCP9600"
#define PLUGIN_VALUENAME1_199 "Temperature"

#define PLUGIN_199_MCP9600_DEVICE "MCP9600"

enum MCP_ChipId {
  Unknown_DEVICE = 0,
  MCP9600_DEVICE = 0x60
};


struct P199_sensordata {
  P199_sensordata() :
    last_hum_val(0.0),
    last_press_val(0.0),
    last_temp_val(0.0),
    last_dew_temp_val(0.0),
    last_measurement(0),
    sensorID(Unknown_DEVICE),
    i2cAddress(0),
    state(BMx_Uninitialized) {}

    byte get_config_settings() const {
      switch (sensorID) {
        case BMP280_DEVICE_SAMPLE1:
        case BMP280_DEVICE_SAMPLE2:
        case BMP280_DEVICE:  return 0x28; // Tstandby 62.5ms, filter 4, 3-wire SPI Disable
        case BME280_DEVICE:  return 0x28; // Tstandby 62.5ms, filter 4, 3-wire SPI Disable
        default: return 0;
      }
    }

    byte get_control_settings() const {
      switch (sensorID) {
        case BMP280_DEVICE_SAMPLE1:
        case BMP280_DEVICE_SAMPLE2:
        case BMP280_DEVICE:  return 0x93; // Oversampling: 8x P, 8x T, normal mode
        case BME280_DEVICE:  return 0x93; // Oversampling: 8x P, 8x T, normal mode
        default: return 0;
      }
    }

    String getFullDeviceName() const {
      String devicename = getDeviceName();
      if (sensorID == BMP280_DEVICE_SAMPLE1 ||
          sensorID == BMP280_DEVICE_SAMPLE2)
      {
        devicename += " sample";
      }
      return devicename;
    }

    String getDeviceName() const {
      switch (sensorID) {
        case BMP280_DEVICE_SAMPLE1:
        case BMP280_DEVICE_SAMPLE2:
        case BMP280_DEVICE:  return PLUGIN_199_BMP280_DEVICE;
        case BME280_DEVICE:  return PLUGIN_199_BME280_DEVICE;
        default: return "Unknown";
      }
    }

    boolean hasHumidity() const {
      switch (sensorID) {
        case BMP280_DEVICE_SAMPLE1:
        case BMP280_DEVICE_SAMPLE2:
        case BMP280_DEVICE:  return false;
        case BME280_DEVICE:  return true;
        default: return false;
      }
    }

    bool initialized() const {
      return state != BMx_Uninitialized;
    }

    void setUninitialized() {
      state = BMx_Uninitialized;
    }

  bme280_uncomp_data uncompensated;
  bme280_calib_data calib;
  float last_hum_val;
  float last_press_val;
  float last_temp_val;
  float last_dew_temp_val;
  unsigned long last_measurement;
  BMx_ChipId sensorID;
  uint8_t i2cAddress;
  unsigned long moment_next_step = 0;
  BMx_state state;
};

std::map<uint8_t, P199_sensordata> P199_sensors;

int Plugin_199_i2c_addresses[2] = { 0x60, 0x61 };

uint8_t Plugin_199_i2c_addr(struct EventStruct *event) {
  uint8_t i2cAddress = static_cast<uint8_t>(PCONFIG(0));
  if (i2cAddress != Plugin_199_i2c_addresses[0] && i2cAddress != Plugin_199_i2c_addresses[1]) {
    // Set to default address
    i2cAddress = Plugin_28_i2c_addresses[0];
  }
  if (P199_sensors.count(i2cAddress) == 0) {
    P199_sensors[i2cAddress] = P199_sensordata();
  }
  return i2cAddress;
}

boolean Plugin_199(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_199;
        Device[deviceCount].Type = DEVICE_TYPE_I2C;
        Device[deviceCount].VType = SENSOR_TYPE_TEMP_HUM_BARO;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = 3;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_199);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_199));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_199));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_199));
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {
        const uint8_t i2cAddress = Plugin_199_i2c_addr(event);
        P199_sensordata& sensor = P199_sensors[i2cAddress];
        addFormSelectorI2C(F("p199_bme280_i2c"), 2, Plugin_28_i2c_addresses, i2cAddress);
        if (sensor.sensorID != Unknown_DEVICE) {
          String detectedString = F("Detected: ");
          detectedString += sensor.getFullDeviceName();
          addUnit(detectedString);
        }
        addFormNote(F("SDO Low=0x76, High=0x77"));

        addFormNumericBox(F("Altitude"), F("p199_bme280_elev"), PCONFIG(1));
        addUnit(F("m"));

        addFormNumericBox(F("Temperature offset"), F("p199_bme280_tempoffset"), PCONFIG(2));
        addUnit(F("x 0.1C"));
        String offsetNote = F("Offset in units of 0.1 degree Celcius");
        if (sensor.hasHumidity()) {
          offsetNote += F(" (also correct humidity)");
        }
        addFormNote(offsetNote);

        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        const uint8_t i2cAddress = getFormItemInt(F("p199_bme280_i2c"));
        Plugin_199_check(i2cAddress); // Check id device is present
        PCONFIG(0) = i2cAddress;
        PCONFIG(1) = getFormItemInt(F("p199_bme280_elev"));
        PCONFIG(2) = getFormItemInt(F("p199_bme280_tempoffset"));
        success = true;
        break;
      }
    case PLUGIN_ONCE_A_SECOND:
      {
        const uint8_t i2cAddress = Plugin_199_i2c_addr(event);
        const float tempOffset = PCONFIG(2) / 10.0;
        if (Plugin_199_update_measurements(i2cAddress, tempOffset, event->TaskIndex)) {
          // Update was succesfull, schedule a read.
          schedule_task_device_timer(event->TaskIndex, millis() + 10);
        }
        break;
      }

    case PLUGIN_READ:
      {
        const uint8_t i2cAddress = Plugin_199_i2c_addr(event);
        P199_sensordata& sensor = P199_sensors[i2cAddress];
        if (sensor.state != BMx_New_values) {
          success = false;
          break;
        }
        sensor.state = BMx_Values_read;
        if (!sensor.hasHumidity()) {
          // Patch the sensor type to output only the measured values.
          event->sensorType = SENSOR_TYPE_TEMP_EMPTY_BARO;
        }
        UserVar[event->BaseVarIndex] = sensor.last_temp_val;
        UserVar[event->BaseVarIndex + 1] = sensor.last_hum_val;
        const int elev = PCONFIG(1);
        if (elev) {
           UserVar[event->BaseVarIndex + 2] = Plugin_199_pressureElevation(sensor.last_press_val, elev);
        } else {
           UserVar[event->BaseVarIndex + 2] = sensor.last_press_val;
        }
        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          log.reserve(40); // Prevent re-allocation
          log = sensor.getDeviceName();
          log += F(" : Address: 0x");
          log += String(i2cAddress,HEX);
          addLog(LOG_LEVEL_INFO, log);
          log = sensor.getDeviceName();
          log += F(" : Temperature: ");
          log += UserVar[event->BaseVarIndex];
          addLog(LOG_LEVEL_INFO, log);
          if (sensor.hasHumidity()) {
            log = sensor.getDeviceName();
            log += F(" : Humidity: ");
            log += UserVar[event->BaseVarIndex + 1];
            addLog(LOG_LEVEL_INFO, log);
          }
          log = sensor.getDeviceName();
          log += F(" : Barometric Pressure: ");
          log += UserVar[event->BaseVarIndex + 2];
          addLog(LOG_LEVEL_INFO, log);
        }
        success = true;
        break;
      }
      case PLUGIN_EXIT:
      {
        const uint8_t i2cAddress = Plugin_199_i2c_addr(event);
        P199_sensors.erase(i2cAddress);
        break;
      }
  }
  return success;
}


// Only perform the measurements with big interval to prevent the sensor from warming up.
bool Plugin_199_update_measurements(const uint8_t i2cAddress, float tempOffset, unsigned long task_index) {
  P199_sensordata& sensor = P199_sensors[i2cAddress];
  const unsigned long current_time = millis();
  Plugin_199_check(i2cAddress); // Check id device is present
  if (!sensor.initialized()) {
    if (!Plugin_199_begin(i2cAddress)) {
      return false;
    }
    sensor.state = BMx_Initialized;
    sensor.last_measurement = 0;
  }
  if (sensor.state != BMx_Wait_for_samples) {
    if (sensor.last_measurement != 0 &&
        !timeOutReached(sensor.last_measurement + (Settings.TaskDeviceTimer[task_index] * 1000))) {
      // Timeout has not yet been reached.
      return false;
    }

    sensor.last_measurement = current_time;
    // Set the Sensor in sleep to be make sure that the following configs will be stored
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, 0x00);
    if (sensor.hasHumidity()) {
      I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROLHUMID, BME280_CONTROL_SETTING_HUMIDITY);
    }
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONFIG, sensor.get_config_settings());
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, sensor.get_control_settings());
    sensor.state = BMx_Wait_for_samples;
    return false;
  }

  // It takes at least 1.587 sec for valit measurements to complete.
  // The datasheet names this the "T63" moment.
  // 1 second = 63% of the time needed to perform a measurement.
  if (!timeOutReached(sensor.last_measurement + 1587)) {
    return false;
  }
  if (!Plugin_199_readUncompensatedData(i2cAddress)) {
    return false;
  }
  // Set to sleep mode again to prevent the sensor from heating up.
  I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, 0x00);

  sensor.last_measurement = current_time;
  sensor.state = BMx_New_values;
  sensor.last_temp_val = Plugin_199_readTemperature(i2cAddress);
  sensor.last_press_val = ((float)Plugin_199_readPressure(i2cAddress)) / 100;
  sensor.last_hum_val = ((float)Plugin_199_readHumidity(i2cAddress));


  String log;
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    log.reserve(120); // Prevent re-allocation
    log = sensor.getDeviceName();
    log += F(":");
  }
  boolean logAdded = false;
  if (sensor.hasHumidity()) {
    // Apply half of the temp offset, to correct the dew point offset.
    // The sensor is warmer than the surrounding air, which has effect on the perceived humidity.
    sensor.last_dew_temp_val = compute_dew_point_temp(sensor.last_temp_val + (tempOffset / 2.0), sensor.last_hum_val);
  } else {
    // No humidity measurement, thus set dew point equal to air temperature.
    sensor.last_dew_temp_val = sensor.last_temp_val;
  }
  if (tempOffset > 0.1 || tempOffset < -0.1) {
    // There is some offset to apply.
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" Apply temp offset ");
      log += tempOffset;
      log += F("C");
    }
    if (sensor.hasHumidity()) {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F(" humidity ");
        log += sensor.last_hum_val;
      }
      sensor.last_hum_val = compute_humidity_from_dewpoint(sensor.last_temp_val + tempOffset, sensor.last_dew_temp_val);
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F("% => ");
        log += sensor.last_hum_val;
        log += F("%");
      }
    } else {
      sensor.last_hum_val = 0.0;
    }
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" temperature ");
      log += sensor.last_temp_val;
    }
    sensor.last_temp_val = sensor.last_temp_val + tempOffset;
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F("C => ");
      log += sensor.last_temp_val;
      log += F("C");
      logAdded = true;
    }
  }
  if (sensor.hasHumidity()) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" dew point ");
      log += sensor.last_dew_temp_val;
      log += F("C");
      logAdded = true;
    }
  }
  if (logAdded && loglevelActiveFor(LOG_LEVEL_INFO))
    addLog(LOG_LEVEL_INFO, log);
  return true;
}


//**************************************************************************/
// Check BME280 presence
//**************************************************************************/
bool Plugin_199_check(uint8_t i2cAddress) {
  bool wire_status = false;
  const uint8_t chip_id = I2C_read8_reg(i2cAddress, BMx280_REGISTER_CHIPID, &wire_status);
  P199_sensordata& sensor = P199_sensors[i2cAddress];
  if (!wire_status) sensor.setUninitialized();
  switch (chip_id) {
    case BMP280_DEVICE_SAMPLE1:
    case BMP280_DEVICE_SAMPLE2:
    case BMP280_DEVICE:
    case BME280_DEVICE: {
      if (wire_status) {
        // Store detected chip ID when chip found.
        if (sensor.sensorID != chip_id) {
          sensor.sensorID = static_cast<BMx_ChipId>(chip_id);
          sensor.setUninitialized();
          String log = F("BMx280 : Detected ");
          log += sensor.getFullDeviceName();
          addLog(LOG_LEVEL_INFO, log);
        }
      } else {
        sensor.sensorID = Unknown_DEVICE;
      }
      break;
    }
    default:
      sensor.sensorID = Unknown_DEVICE;
      break;
  }
  if (sensor.sensorID == Unknown_DEVICE) {
    String log = F("BMx280 : Unable to detect chip ID");
    addLog(LOG_LEVEL_INFO, log);
    return false;
  }
  return wire_status;
}

//**************************************************************************/
// Initialize BME280
//**************************************************************************/
bool Plugin_199_begin(uint8_t i2cAddress) {
  if (! Plugin_199_check(i2cAddress))
    return false;
  // Perform soft reset
  I2C_write8_reg(i2cAddress, BMx280_REGISTER_SOFTRESET, 0xB6);
  delay(2);  // Startup time is 2 ms (datasheet)
  Plugin_199_readCoefficients(i2cAddress);
//  delay(65); //May be needed here as well to fix first wrong measurement?
  return true;
}


bool Plugin_199_readUncompensatedData(uint8_t i2cAddress) {
  // wait until measurement has been completed, otherwise we would read
  // the values from the last measurement
  if (I2C_read8_reg(i2cAddress, BMx280_REGISTER_STATUS) & 0x08)
    return false;

  I2Cdata_bytes BME280_data(BME280_P_T_H_DATA_LEN, BME280_DATA_ADDR);
  bool allDataRead = I2C_read_bytes(i2cAddress, BME280_data);
  if (!allDataRead) {
    return false;
  }
  /* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

  P199_sensordata& sensor = P199_sensors[i2cAddress];

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)BME280_data[BME280_DATA_ADDR + 0] << 12;
	data_lsb = (uint32_t)BME280_data[BME280_DATA_ADDR + 1] << 4;
	data_xlsb = (uint32_t)BME280_data[BME280_DATA_ADDR + 2] >> 4;
	sensor.uncompensated.pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)BME280_data[BME280_DATA_ADDR + 3] << 12;
	data_lsb = (uint32_t)BME280_data[BME280_DATA_ADDR + 4] << 4;
	data_xlsb = (uint32_t)BME280_data[BME280_DATA_ADDR + 5] >> 4;
	sensor.uncompensated.temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_lsb = (uint32_t)BME280_data[BME280_DATA_ADDR + 6] << 8;
	data_msb = (uint32_t)BME280_data[BME280_DATA_ADDR + 7];
	sensor.uncompensated.humidity = data_msb | data_lsb;
  return true;
}

//**************************************************************************/
// Read temperature
//**************************************************************************/
float Plugin_199_readTemperature(uint8_t i2cAddress)
{
  P199_sensordata& sensor = P199_sensors[i2cAddress];
  int32_t var1, var2;
  int32_t adc_T = sensor.uncompensated.temperature;
  var1  = ((((adc_T >> 3) - ((int32_t)sensor.calib.dig_T1 << 1))) *
           ((int32_t)sensor.calib.dig_T2)) >> 11;

  var2  = (((((adc_T >> 4) - ((int32_t)sensor.calib.dig_T1)) *
             ((adc_T >> 4) - ((int32_t)sensor.calib.dig_T1))) >> 12) *
           ((int32_t)sensor.calib.dig_T3)) >> 14;

  sensor.calib.t_fine = var1 + var2;

  float T  = (sensor.calib.t_fine * 5 + 128) >> 8;
  return T / 100;
}

