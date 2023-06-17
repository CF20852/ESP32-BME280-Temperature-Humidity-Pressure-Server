# ESP32-BME280-Temperature-Humidity-Pressure-Server
This web server was prototyped on an Adafruit ESP32 Feather V2 with a SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic), using the Arduino Version 2.1.0 IDE.  The ESP32 board is connected to the BME280 using an I2C interface.  The BME280 is at address 0x77.

<h3>Some features of this software:<br></h3>
There are two web pages:

* The page at <ip address>/current gives you the current temperature (deg. F), relative humidity (%), dew point (deg. F), and barometric pressure (hPa) readings.<br><br>
* The page at <ip address>/ (root) gives you a plot of the temperature, relative humidity, dew point, and pressure for up to the past three days.<br><p>

Other features:<br>
* This server uses Unix time (seconds since midnight January 1, 1970) to time-tag the samples of temperature, humidity, dew point, and pressure.  It uses an internal timer in the ESP32 to measure time.<br><br>
* The server sets its real-time clock over the web (using NTP servers) when it starts up, and every three days plus a random number of seconds up to about four hours thereafter.<br><br>
* The server uses chunked data transmission to transfer data to the client in JSON format.  Using chunked transmission means we don't have to store the entire JSON response, which can be up to 1440 samples of five parameters per sample.<br><p>

<h3>Libraries:</h3>

The following Arduino libraries are required:<p>
* Arduinojson by Benoit Blanchon (v. 6.21.2 was used)<br>
* ESP32AsyncWebSrv by dvarrel (v. 1.2.6 was used)<br>
* ESPAsyncTCP by dvarrel (v. 1.2.4 was used)<br>
* SparkFun BME280 by SparkFun Electronics (v. 2.0.9 was used)<br>
* ESP32Time by fbiego (v. 2.0.0 was used)<br>
* WiFi<br>
* SPIFFS</p>

<h3>Process</h3>
Use Arduino 1.8.19 with the ESP32 Sketch Data Upload tool that you can install using the instructions in https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/ under the "Installing the Arduino ESP32 Filesystem Uploader" heading to upload the data folder and the index.html file it contains to the ESP32 SPIFFS memory.  You may wish to customize the index.html file for your application first.<p>
Install any Arduino libraries listed above that you don't already have in your IDE.<p>
Change the Wi-Fi ssid and password in lines 13 and 14 of the .ino file to your network ssid and password.  Build and upload the .ino file to your ESP32 board using the Arduino IDE.
