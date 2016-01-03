# DHTBLESensor
Arduino 101 Sketch to deliver temperature and humidity from a DHT22/RHT03 sensor over BLE.

The Sketch is based on Intel's CurieBLE examples and Rob Tillaart's DHT22 library example.

## Parts
 1. Arduino 101, available at https://www.arduino.cc/en/Main/ArduinoBoard101
 1. RHT03 (DHT22) environmental temperature and humidity sensor, available [from Sparkfun](https://www.sparkfun.com/products/10167).
 1. M/F Jumper Wires to connect the two, available [from Sparkfun](https://www.sparkfun.com/products/9140).

## Wiring
DHT22/RHT03 wiring is available at the Sparkfun link above.

Connect
- DHT22 pin 1 (Vcc) to Arduino 3.3V
- DHT22 pin 2 (Signal) to Arduino pin 7 (DHT22_DATA)
- DHT22 pin 3 and 4 to Arduino GND.  Both pins are grounded because some versions of the DHT22 expect ground on pin 3 instead of pin 4.

## To use
Values can be read using any device with appropriate BLE software and hardware.
For example, an Android phone running the [Nordic Android BLE explorer](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp).
