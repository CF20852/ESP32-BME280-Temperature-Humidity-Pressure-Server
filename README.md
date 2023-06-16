# ESP32-BME280-Temperature-Humidity-Pressure-Server
This web server was prototyped on an Adafruit ESP32 Feather V2 with a Sparkfun BME280, using the Arduino Version 2.1.0 IDE.  The ESP32 board is connected to the BME280 using an I2C interface.  The BME280 is at address 0x77.

The following Arduino libraries are required:
    Arduinojson by Benoit Blanchon (v. 6.21.2 was used)<br>
    ESP32AsyncWebSrv by dvarrel (v. 1.2.6 was used)
    ESPAsyncTCP by dvarrel (v. 1.2.4 was used)
    SparkFun BME280 by SparkFun Electronics (v. 2.0.9 was used)
    ESP32Time by fbiego (v. 2.0.0 was used)
    WiFi
    SPIFFS

Use Arduino 1.8.19 with the ESP32 Sketch Data Upload tool that you can install using the instructions in https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/ under the "Installing the Arduino ESP32 Filesystem Uploader" heading to upload the data folder and the index.html file it contains to the ESP32 SPIFFS memory.  You may wish to customize the index.html file for your application first.
Install any Arduino libraries listed above that you don't already have in your IDE.
Change the Wi-Fi ssid and password in lines 13 and 14 of the .ino file to your network ssid and password.  Build and upload the .ino file to your ESP32 board using the Arduino IDE.
