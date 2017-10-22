/*
 * Environmental Temperature and Humidity BLE Service for Arduino 101.
 * Based on Intel's CurieBLE Battery Monitor Example (LGPL)
 * and on Rob Tillaart's DHT library (Public Domain) http://playground.arduino.cc/Main/DHTLib
 * 
 * Copyright (c) 2016, 2017 Bradford Needham
 * { @bneedhamia , https://www.needhamia.com, https://github.com/bneedhamia }
 *
 * Licensed under LGPL 2.1 (to match the CurieBLE example),
 * a copy of which should have been supplied with this file.
 * 
 * This code implements
 *   The BLE Environmental Sensing Service (see https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx)
 *   The Humidity and Temperature Characteristics (see https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.environmental_sensing.xml)
 *   
 * The values can be read from any device running an appropriate BLE Client application.
 * For example, for Android: https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp
 * 
 * RHT03 parts and datasheets are available from Sparkfun: https://www.sparkfun.com/products/10167
 * Male/Female jumper wires for connecting the RHT02 (DHT22) are available from Sparkfun: https://www.sparkfun.com/products/9140
 */
#include <CurieBLE.h>
#include <dht.h> // DHT22 library, modified for Arduino 101.  Source at https://github.com/bneedhamia/DHT

/*
 * Pins:
 *   DHT22 pin 1 --> power input, connected to Arduino 5V
 *     Pin 1 is the left pin as the open grid faces you and the pins point down.
 *   DHT22 pin 2 --> signal output to Arduino (connect to DHT22_DATA)
 *   DHT22 pin3 & 4 --> Arduino GND. Both 3 and 4 are grounded because
 *     some DHT22's expect pin 3 to be grounded, while others expect pin 4 to be grounded.
 *     
 *   PIN_CONNECTED_LED = Output. HIGH (lighted) if somebody is connected to our BLE Service.
 *   
 *   Connect a 4.7K ohm resistor between DHT22 pin 2 and Arduino 5V.
 *   
 */
const int PIN_CONNECTED_LED = 13;
const int DHT22_DATA = 7;

const char *BLE_LOCAL_NAME = "TempHumidRHT03";  // Advertised name of our BLE Device
const long TEMPERATURE_UPDATE_INTERVAL_MS = 2000L;  // minimum time between temperature & humidity updates, in milliseconds.

BLEPeripheral myBLE;              // Root of our BLE Peripheral (server) capability
BLEService bleEnvironmental("181A"); // Our BLE Service.  See https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.environmental_sensing.xml
BLEUnsignedShortCharacteristic bleHumidityPC("2A6F", BLERead | BLENotify); // see https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.humidity.xml
  // NOTE: It's an unsigned short with implied exponent of 10^-2.  For example a value of 2563 = 25.63%
BLEShortCharacteristic bleTemperatureC("2A6E", BLERead | BLENotify); // see https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature.xml
  // NOTE: It's a short with implied exponent of 10^-2.  For example a value of 2321 = 23.21 degrees Celsius

dht dht22;    // manages I/O with the DHT22 temperature/Humidity sensor. Caches temperature(C) and Humidity(%).
long whenSensedMs = 0;       // time (ms since startup) when the sensor was last read.
unsigned short oldEncodedHumidity = 0.0 * 100.0;  // previous value of the BLE Characteristic for Humidity
short oldEncodedTemperature = -200.0 * 100.0;     // same for temperature.

void setup() {
  Serial.begin(9600);
  
  pinMode(PIN_CONNECTED_LED, OUTPUT);   // initially we're not connected to anyone.
  digitalWrite(PIN_CONNECTED_LED, LOW);
  // DHT22_DATA is managed by the DHT library.

  // Configure the BLE stack
  myBLE.setLocalName(BLE_LOCAL_NAME);
  myBLE.setAdvertisedServiceUuid(bleEnvironmental.uuid());
  myBLE.addAttribute(bleEnvironmental);
  myBLE.addAttribute(bleHumidityPC);
  myBLE.addAttribute(bleTemperatureC);

  // Initialize our BLE Characteristics
  readTemperatureAndHumidity();
  setBLETemperatureAndHumidity(dht22.temperature, dht22.humidity);

  // Serial.println(F("Starting BLE Service"));  // Because of Curie startup, no serial output will be seen until the loop.
  myBLE.begin();
}

void loop() {
  BLECentral central = myBLE.central();

  if (central) {
    digitalWrite(PIN_CONNECTED_LED, HIGH);

    Serial.print("Connected to central: ");
    Serial.println(central.address());

    //XXX change this into a state machine.
    while (central.connected()) {  //XXX Not a good idea because connecting hangs all other Arduino functions (like reset and download)
      long nowMs = millis();
      if ((long)(nowMs - whenSensedMs) >= TEMPERATURE_UPDATE_INTERVAL_MS) {        
        readTemperatureAndHumidity();
        setBLETemperatureAndHumidity(dht22.temperature, dht22.humidity);
      }
    }
    // Disconnected

    digitalWrite(PIN_CONNECTED_LED, LOW);
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

/*
 * Reads the current temperature and humidity from the RHT03 sensor,
 * updating the corresponding global variables
 * and the time of the reading.
 * 
 * If not enough time has passed since the last reading, the values will not be changed.
 */
void readTemperatureAndHumidity() {
  long nowMs = millis();
  if (nowMs - whenSensedMs < TEMPERATURE_UPDATE_INTERVAL_MS) {
    return;   // not ready to be read yet.
  }
  whenSensedMs = nowMs;
  
  int result = dht22.read22(DHT22_DATA);
  
  if (result != DHTLIB_OK) {
    if (result == DHTLIB_ERROR_TIMEOUT) {
      Serial.println("Timeout waiting for DHT22");
    } else if (result == DHTLIB_ERROR_CHECKSUM) {
      Serial.println("Garbled result from DHT22");
    } else {
      Serial.println("Unknown failure from DHT22");
    }
    return; // try again later.
  }

  // Cached results are now available in dht22.humidity and dht22.temperature
}


/*
 * Copies the latest temperature and humidity values read from the sensor
 * into the corresponding BLE Characteristics,
 * translating the format as necessary.
 * TODO add the temperature and humidity as parameters rather than using globals.
 * Manage your own temp and hum encoded values.
 * 
 * Parameters:
 *   temperatureC = the temperature (degrees Celsius) to set to.
 *   humidityPC = the relative humidity (percent) to set to.
 */
void setBLETemperatureAndHumidity(double temperatureC, double humidityPC) {
  //TODO Modify to keep the old and new values, so you can setValue only if they've changed.
  unsigned short newHumidity = (unsigned short) (humidityPC * 100.0 + 0.5);
  //if (newHumidity != ....) {
    bleHumidityPC.setValue(newHumidity);
  //}

  short newTemperatureC = (short) (temperatureC * 100.0 + 0.5);
  bleTemperatureC.setValue(newTemperatureC);
}

