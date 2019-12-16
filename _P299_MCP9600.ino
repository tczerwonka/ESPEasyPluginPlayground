//#######################################################################################################
//######################### Plugin 299: MCP9600 thermocouple to temperature converter ###################
//#######################################################################################################
// mostly swiped from
// https://github.com/sparkfun/SparkFun_MCP9600_Arduino_Library/blob/master/src/SparkFun_MCP9600.cpp
//#######################################################################################################

#define PLUGIN_299
#define PLUGIN_ID_299                         299
#define PLUGIN_NAME_299                       "MCP9600 thermocouple to temperature"
#define PLUGIN_VALUENAME1_299                 "TEMPERATURE"

#define MCP9600_ADDRESS                     (0x60)  // default address 
#define MCP9600_CMD_RANGE_COMMAND           (0x51)

//
#define MCP9600_DEV_ID_UPPER 0x40 //value of the upper half of the device ID register. lower half is used for device revision
#define MCP9600_DEV_RESOLUTION 0.0625 //device resolution (temperature in C that the LSB represents)
#define MCP9600_retryAttempts 3 //how many times to attempt to read a register from the thermocouple before giving up

// register pointers for various device functions
enum MCP9600_Register: uint8_t {
  HOT_JUNC_TEMP = 0x00,
  DELTA_JUNC_TEMP = 0x01,
  COLD_JUNC_TEMP = 0x02,
  RAW_ADC = 0x03,
  SENSOR_STATUS = 0x04,
  THERMO_SENSOR_CONFIG = 0x05,
  DEVICE_CONFIG = 0x06,
  ALERT1_CONFIG = 0x08,
  ALERT2_CONFIG = 0x09,
  ALERT3_CONFIG = 0x0A,
  ALERT4_CONFIG = 0x0B,
  ALERT1_HYSTERESIS = 0x0C,
  ALERT2_HYSTERESIS = 0x0D,
  ALERT3_HYSTERESIS = 0x0E,
  ALERT4_HYSTERESIS = 0x0F,
  ALERT1_LIMIT = 0x10,
  ALERT2_LIMIT = 0x11,
  ALERT3_LIMIT = 0x12,
  ALERT4_LIMIT = 0x13,
  DEVICE_ID = 0x20,
};

enum MCP9600_Thermocouple_Type: uint8_t {
  TYPE_K = 0b000,
  TYPE_J = 0b001,
  TYPE_T = 0b010,
  TYPE_N = 0b011,
  TYPE_S = 0b100,
  TYPE_E = 0b101,
  TYPE_B = 0b110,
  TYPE_R = 0b111,
};

enum MCP9600_Ambient_Resolution: bool {
  RES_ZERO_POINT_0625 = 0,
  RES_ZERO_POINT_25 = 1,
};

enum MCP9600_Thermocouple_Resolution: uint8_t {
  RES_18_BIT = 0b00,
  RES_16_BIT = 0b01,
  RES_14_BIT = 0b10,
  RES_12_BIT = 0b11,
};

enum MCP9600_Burst_Sample: uint8_t {
  SAMPLES_1 = 0b000,
  SAMPLES_2 = 0b001,
  SAMPLES_4 = 0b010,
  SAMPLES_8 = 0b011,
  SAMPLES_16 = 0b100,
  SAMPLES_32 = 0b101,
  SAMPLES_64 = 0b110,
  SAMPLES_128 = 0b111,
};

enum MCP9600_Shutdown_Mode: uint8_t {
  NORMAL = 0x00,
  SHUTDOWN = 0x01,
  BURST = 0x02,
};
//

boolean Plugin_299(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {

    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_299;
        Device[deviceCount].Type = DEVICE_TYPE_I2C;
        Device[deviceCount].VType = SENSOR_TYPE_SINGLE;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = 1;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_299);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_299));
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {     
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        success = true;
        break;
      }

    case PLUGIN_INIT:
      {
        Plugin_299_begin();
        success = true;
        break;
      }

    case PLUGIN_READ:
      {
        uint16_t value;
        value = Plugin_299_getDistance();        
        UserVar[event->BaseVarIndex] = value;
        String log = F("P299 : distance = ");
        log += value;
        log += F(" cm");
        addLog(LOG_LEVEL_INFO,log);
        success = true;
        break;
      }
  }
  return success;
}

//**************************************************************************/
// Sensor setup
//**************************************************************************/
void Plugin_299_begin(void) 
{
}


//**************************************************************************/
//
//**************************************************************************/
boolean Plugin_299_available() {
	uint8_t status = Plugin_299_readSingleRegister(SENSOR_STATUS);
	return bitRead(status, 6);
}



//**************************************************************************/
// I2C abstraction
//**************************************************************************/
uint8_t Plugin_299_readSingleRegister(MCP9600_Register reg){
	//Attempt to read the register until we exit with no error code
	//This attempts to fix the bug where clock stretching sometimes failes, as
	//described in the MCP9600 eratta
	for(uint8_t attempts; attempts <= retryAttempts; attempts++){
		Wire.beginTransmission(MCP9600_ADDRESS);
		Wire.write(reg);
		Wire.endTransmission();
		if(Wire.requestFrom(MCP9600_ADDRESS, 1) != 0){
      			return Wire.read();     
    		}
  	}
}




//**************************************************************************/
// Report distance
//**************************************************************************/
uint16_t Plugin_299_getDistance() 
{
  uint16_t value = 0;
    
  Wire.beginTransmission(MCP9600_ADDRESS);
  Wire.write(MCP9600_CMD_RANGE_COMMAND);
  Wire.endTransmission();
  
  delay(70);                                                                      // transmit -> receive turnaround time (up to 65ms)

  Wire.requestFrom(MCP9600_ADDRESS, 2);
  if (Wire.available() > 1) {
    value = ((Wire.read() << 8) | Wire.read());
  }
  
  return value;
}
