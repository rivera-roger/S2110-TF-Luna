/*    
 *     TF Luna Lidar Sensor integration with Seeed Studio SenseCAP S2110
 *     S2110 is a Groove Sensor (I2C) to RS-485 based on XIAO RP2040
 *     Use TFLI2C Library by Bud Ryerson
 *     Use Preferences Library by Volodymyr Shymanskyy to save Modbus Address
 *     by: Roger Rivera
 *     March 2023
 */
 
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#include <Preferences.h>

#include <Wire.h>        
#include <TFLI2C.h>      // TFLuna-I2C Library 

#define GROVE_SWITCH_PIN D10
#define SENSOR_ANALOG_PIN A3
#define SENSOR_ANALOG_E_PIN A2
#define SENSOR_DIGITAL_PIN D3
#define SENSOR_DIGITAL_E_PIN D2
#define SENSOR_IIC_SCL_PIN SCL
#define SENSOR_IIC_SDA_PIN SDA

#define GROVE_SWITCH_IIC                 \
    pinMode(GROVE_SWITCH_PIN, OUTPUT);   \
    digitalWrite(GROVE_SWITCH_PIN, LOW); \
    delay(10)

Preferences prefs;

TFLI2C tflI2C;

int16_t  tfDist = 0;    // distance in centimeters
int16_t  tfFlux = 0 ;   // signal quality in arbitrary units
int16_t  tfTemp = 0 ;   // temperature in 0.01 degree Celsius

int16_t  tfAddr = TFL_DEF_ADR;  // Use this default I2C address or
                                // set variable to your own value
uint16_t tfFrame = TFL_DEF_FPS;
                                
int16_t baudrate = 9600;
int16_t ModbusAddress = 1;
int16_t ModbusAddressAux = 1;

const int numHoldingRegisters = 4;
const int numInputRegisters = 4;

void setup()
{
    GROVE_SWITCH_IIC;
    Serial.begin( 115200);  // Initialize serial port
    Wire.begin();           // Initialize Wire library
    prefs.begin("S2110");

    // retrieve ModbusAddress from Preferences
    ModbusAddress = prefs.getShort("ModbusAddress",1);
    
    Serial.print("ModbusAddress: ");
    Serial.println(ModbusAddress);
    
    // start the Modbus RTU server, with id = ModbusAddress
    RS485.setDelays(5000, 5000);
    if (!ModbusRTUServer.begin(ModbusAddress, baudrate))
    {
        Serial.println("Failed to start Modbus RTU Client!");
        //while (1);
    }
    ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);
    ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);

    ModbusRTUServer.holdingRegisterWrite(0x00, ModbusAddress);

    // change the Frame Rate of TF Luna
    tfFrame = 10;
    tflI2C.Set_Frame_Rate( tfFrame, tfAddr);
}

void loop()
{
    if( tflI2C.getData( tfDist, tfFlux, tfTemp, tfAddr))
    {
        Serial.print("Dist: ");
        Serial.println(tfDist);          // print the distance
        ModbusRTUServer.holdingRegisterWrite(0x01, tfDist);
        ModbusRTUServer.holdingRegisterWrite(0x02, tfFlux);
        ModbusRTUServer.holdingRegisterWrite(0x03, tfTemp);
    }
    else tflI2C.printStatus();

    // poll for Modbus RTU requests
    ModbusRTUServer.poll();

    // verify if ModbusAddress is changed
    ModbusAddressAux = ModbusRTUServer.holdingRegisterRead(0x00);
    if (ModbusAddress != ModbusAddressAux) {
      ModbusAddress = ModbusAddressAux;
      prefs.putShort("ModbusAddress",ModbusAddress);
    }
    
    Serial.print("ModbusAddress: ");
    Serial.println(ModbusAddress);
    
    delay(500);
}
