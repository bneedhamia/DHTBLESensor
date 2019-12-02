![The Circuit Wiring](https://github.com/bneedhamia/DHTBLESensor/blob/master/circuit.jpg)
# DHTBLESensor
Arduino 101 Sketch to deliver temperature and humidity from a DHT22/RHT03 sensor over BLE.

NOTE: Intel discontinued the Arduino 101 (Curie) board in 2017, so this project is obsolete.

The Sketch is based on Intel's CurieBLE examples and Rob Tillaart's DHT22 library example.

## Parts
 1. Arduino 101, available at https://www.arduino.cc/en/Main/ArduinoBoard101
 1. RHT03 (DHT22) environmental temperature and humidity sensor, available [from Sparkfun](https://www.sparkfun.com/products/10167).
 1. 4.7K Ohm resistor.

## Wiring
See circuit.jpg

Connect
- DHT22 pin 1 (Vcc) to Arduino 5V.
- DHT22 pin 2 (Signal) to Arduino pin 7 (DHT22_DATA)
- DHT22 pin 3 and 4 to Arduino GND.  Both pins are grounded because some versions of the DHT22 expect ground on pin 3 instead of pin 4.
- A 4.7K pull-up resistor between DHT22 pin 2 and 5V.

## To use
Values can be read using any device with appropriate BLE software and hardware.
For example, an Android phone running the [Nordic Android BLE explorer](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp).
