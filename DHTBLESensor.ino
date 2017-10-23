/*
   Environmental Temperature and Humidity BLE Service for Arduino 101.
   Based on Intel's CurieBLE Battery Monitor Example (LGPL)
   and on Rob Tillaart's DHT library (Public Domain)
   at http://playground.arduino.cc/Main/DHTLib

   Copyright (c) 2016, 2017 Bradford Needham
   { @bneedhamia , https://www.needhamia.com, https://github.com/bneedhamia }

   Licensed under LGPL 2.1 (to match the CurieBLE example),
   a copy of which should have been supplied with this file.

   This code implements the following BLE (Bluetooth Low Energy) objects:
     The BLE Environmental Sensing Service
       (see https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx)
     The Humidity and Temperature Characteristics
       (see https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.environmental_sensing.xml)

   The values can be read from any device running an appropriate
   BLE Client application. For example, for Android:
   https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp

   RHT03 (DHT22) parts and datasheets are available
   from Sparkfun at https://www.sparkfun.com/products/10167
   and Adafruit at https://www.adafruit.com/product/385
*/
//345678901234567892123456789312345678941234567895123456789612345678971234567898

#include <CurieBLE.h>
#include <dht.h> // DHT22 library, modified for Arduino 101. https://github.com/bneedhamia/DHT

/*
   Pins:

   DHT22 pin 1 --> power input, connected to Arduino 5V
     Pin 1 is the left pin as the open grid faces you and the pins point down.
   DHT22 pin 2 --> signal output to Arduino (connect to DHT22_DATA)
   DHT22 pin3 & 4 --> Arduino GND. Both 3 and 4 are grounded
     because some DHT22's expect pin 3 to be grounded, while others
     expect pin 4 to be grounded.

   PIN_CONNECTED_LED = Output. HIGH (lighted) if somebody is
     currently Connected to our BLE Service.

   Wire a 4.7K ohm resistor between DHT22 pin 2 and Arduino 5V.
*/
const int PIN_CONNECTED_LED = 13;
const int DHT22_DATA = 7;

/*
   BLE_LOCAL_NAME = the BLE device name we advertise as ours.
*/
const char *BLE_LOCAL_NAME = "TempHumidRHT03";

// Minimum time between temperature & humidity updates, in milliseconds.
const long TEMPERATURE_UPDATE_INTERVAL_MS = 2000L;

/*
   Our BLE objects:

   myBLE = the root of our BLE Peripheral (server) capability.
   bleEnvironmental = Our BLE Service.
     See https://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.environmental_sensing.xml
   bleHumidityPC =  Our BLE Humidity characteristic.
     See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.humidity.xml
     NOTE: It's an unsigned short with implied exponent of 10^-2.
     For example, a value of 2563 = 25.63%
   bleTemperatureC = Our BLE Temperature characteristic, in degrees C.
     See https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature.xml
     NOTE: It's a short with implied exponent of 10^-2.
     For example a value of 2321 = 23.21 degrees Celsius
   oldEncodedHumidity = previous value of the BLE Humidity Characteristic,
     in the form of the BLE Humidity Characteristic.
     Used to avoid unnecessary updates of the BLE Characteristic.
   oldEncodedTemperature = previous value of the BLE Temperature Characteristic,
     in the form of the BLE Temperature Characteristic.
*/
BLEPeripheral myBLE;
BLEService bleEnvironmental("181A");
BLEUnsignedShortCharacteristic bleHumidityPC("2A6F", BLERead | BLENotify);
BLEShortCharacteristic bleTemperatureC("2A6E", BLERead | BLENotify);
unsigned short oldEncodedHumidity = 0.0 * 100.0;
short oldEncodedTemperature = -200.0 * 100.0;

/*
   dht22 = manages I/O with the DHT22 temperature/Humidity sensor.
     Caches temperature(C) and Humidity(%).
*/
dht dht22;

/*
   Sketch running state variables:

   bleConnected = true if a BLE device is connected to us; false otherwise.
   
   whenSensedMs = time (millis()) when the sensor was last read
     and the Temperature and Humidity sent via BLE.
*/
boolean bleConnected = false;
long whenSensedMs = 0;

// Called automatically on Reset
void setup() {
  Serial.begin(9600);

  pinMode(PIN_CONNECTED_LED, OUTPUT);
  digitalWrite(PIN_CONNECTED_LED, LOW);

  // DHT22_DATA is managed by the DHT library.

  // Configure the BLE stack
  myBLE.setLocalName(BLE_LOCAL_NAME);
  myBLE.setAdvertisedServiceUuid(bleEnvironmental.uuid());
  myBLE.addAttribute(bleEnvironmental);
  myBLE.addAttribute(bleHumidityPC);
  myBLE.addAttribute(bleTemperatureC);

  // Initialize our BLE Characteristics
  whenSensedMs = millis();
  (void) readTemperatureAndHumidity();
  setBLETemperatureAndHumidity(dht22.temperature, dht22.humidity);

  // Because of Curie startup, no serial output will be seen until the loop.
  // Serial.println(F("Starting BLE Service"));
  myBLE.begin();
}

// Automatically called very often, after setup()
void loop() {
  BLECentral central = myBLE.central();
  long nowMs;
  
  // Note the comings and goings of BLE clients (Centrals)
  if (central && central.connected()) {
    if (!bleConnected) {
      bleConnected = true;
      digitalWrite(PIN_CONNECTED_LED, HIGH);
      Serial.print("Connected to central: ");
      Serial.println(central.address());
    }

    // If we wanted to do something only while connected, we'd do it here.
    
  } else {
    if (bleConnected) {
      bleConnected = false;
      digitalWrite(PIN_CONNECTED_LED, LOW);
      Serial.println("Disconnected from central.");
    }
  }

  // Periodically read the sensor, regardless of whether anyone's listening.
  nowMs = millis();
  if ((long)(nowMs - whenSensedMs) >= TEMPERATURE_UPDATE_INTERVAL_MS) {
    whenSensedMs = nowMs;
    if (readTemperatureAndHumidity()) {
      setBLETemperatureAndHumidity(dht22.temperature, dht22.humidity);
    }
  }
}

/*
   Reads the current temperature and humidity from the RHT03 sensor,
   updating the corresponding global variables and
   the time of the reading.

   Returns true if the sensor responded correctly; false if an error occurred.
*/
boolean readTemperatureAndHumidity() {
  int result;
  
  result = dht22.read22(DHT22_DATA);
  if (result != DHTLIB_OK) {
    if (result == DHTLIB_ERROR_TIMEOUT) {
      Serial.println("Timeout waiting for DHT22");
    } else if (result == DHTLIB_ERROR_CHECKSUM) {
      Serial.println("Garbled result from DHT22");
    } else {
      Serial.println("Unknown failure from DHT22");
    }
    return false; // try again later.
  }

  // Cached results are now available in dht22.humidity and dht22.temperature
  return true;
}

/*
   Copies the latest temperature and humidity values read from the sensor
   into the corresponding BLE Characteristics,
   translating the format as necessary.

   Parameters:
     temperatureC = the temperature (degrees Celsius) to set to.
     humidityPC = the relative humidity (percent) to set to.
*/
void setBLETemperatureAndHumidity(double temperatureC, double humidityPC) {
  unsigned short newHumidity = (unsigned short) (humidityPC * 100.0 + 0.5);
  //if (newHumidity != ....) {
  bleHumidityPC.setValue(newHumidity);
  //}

  short newTemperatureC = (short) (temperatureC * 100.0 + 0.5);
  bleTemperatureC.setValue(newTemperatureC);
}

