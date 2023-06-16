/*Copyright (c) 2023 Chip Fleming

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
  
#include <WiFi.h>
#include <SparkFunBME280.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include "time.h"
#include <ESP32Time.h>

BME280 mySensorA;  //Uses default I2C address 0x77

// Replace with your network details
const char *ssid = "your primary WiFi ssid goes here";
const char *password = "your primary WiFi password goes here";

// IP addresses for Wi-Fi initialization
IPAddress local_IP(192, 168, 1, 150);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Cloudflare DNS
IPAddress primarydns(1, 1, 1, 1);
IPAddress secondarydns(1, 0, 0, 1);

// Google DNS
// IPAddress primarydns(8, 8, 8, 8);
// IPAddress secondarydns(8, 8, 4, 4);

// Create AsyncWebServer object on port 8080  // or port 80, or 8008, etc.
AsyncWebServer server(8080);

// Set up the Wi-Fi link
void setupWiFi(void) {
  Serial.println("Initializing Wi-Fi mode...");
  WiFi.mode(WIFI_STA);

  Serial.println("Initializing Wi-Fi IP config...");
  if (!WiFi.config(local_IP, gateway, subnet, primarydns, secondarydns)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected.");

  // Print the IP address
  Serial.println("");
  Serial.print("Connected to WiFi! IP address: ");
  Serial.println(WiFi.localIP());
}

// Set up things to use the hardware timer alarm feature to signal time to take a sample
// We’ll have a timer increment every microsecond, so 180*10^6 increments = 180 seconds = 3 minutes
hw_timer_t * timer0 = NULL;
const uint64_t alarmTimeout = 180UL * 1000000UL;
 
volatile boolean recordSample = false;

void ARDUINO_ISR_ATTR Timer0_ISR() {
  recordSample = true;
}

// Set up a timer to trigger an alarm that will signal that it's time to take a new atmospheric parameter sample
void setupTimer() {
    timer0 = timerBegin(0, 80, true);  // 80 MHz clock / 80 = 1 timer increment/microsecond
    timerAttachInterrupt(timer0, &Timer0_ISR, false);  // timer 0 will cause an interrupt served by Timer0_ISR
    timerAlarmWrite(timer0, alarmTimeout, true);  // timer 0 will auto-reload
    timerAlarmEnable(timer0);
}

//Initialize SPIFFS, which will store this server's web page
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}

// Initialize the BME280 sensor
void initBME() {
  Wire.begin();

  mySensorA.setI2CAddress(0x77);  //The default for the SparkFun Environmental Combo board is 0x77 (jumper 
open).
  //If you close the jumper it is 0x76
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  if (mySensorA.beginI2C() == false) {
    Serial.println("Sensor A connect failed");
  } else {
    Serial.println("Sensor A initialization appears to have succeeded.");
  }
}

// Use the ESP32Time library for the real time clock
ESP32Time rtc(0);  // offset in seconds (not needed if using gmtOffset_sec)
const int gmtOffset_sec = -7 * 3600;  // Phoenix, Arizona, USA
const int daylightOffset_sec = 0;     // Arizona doesn't use DST except on the Navajo and Hopi Reservations
const char *ntpServer = "pool.ntp.org";
const unsigned long threeDays = 3 * 86400;

// Function that sets the time using ntp
void setTheClock(void) {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo); 
  }
}

// Variable to save current epoch time
unsigned long epochTime;

// Variable to store the next epoch time at which to set the clock
unsigned long clockResetTime;

// Function to get a hardware random number generator number between 0 and 2^14-1
unsigned long myRandomNo() {
  return esp_random() >> 18;
}

// Arrays to store the samples
#define NSAMPLES 1440
uint64_t timeStamps[NSAMPLES];
double dataPoints[NSAMPLES][4];

/* sampleHeadPtr is initially zero.  It increments after data are written at the buffer location it points to.
Thus it points to the next location to be written.  If the sampleHeadPtr has not rolled over, i.e., gone
through all the numbers and returned to zero, the oldest sample is at 0.  If the sampleHeadPtr has rolled
over, the oldest sample is at (sampleHeadPtr + 1) % NSAMPLES. 

sampleTailPtr is initially zero if sampleHeadPtr has not rolled over from NSAMPLES-1 to zero.  If
sampleHeadPtr has rolled over, sampleTailPtr is initially set to sampleHeadPtr.  It increments
after data are read at the buffer location it points to.  When pulling data from the buffer,
reading should stop when sampleTailPtr is equal to sampleHeadPtr, because the data at sampleHeadPtr
has already been read out and new data has not been written there yet.*/

unsigned int sampleHeadPtr = 0;
unsigned int sampleTailPtr = 0;
boolean rollOver = false;

// Variables for sending chunked chart data
const char *jsonHeader = "{\"enviro_records\":[";
const char *jsonFooter = "]}\r\n";
StaticJsonDocument<256> readings;
char jsonString[256];

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  setupWiFi();

  setupTimer();

  // Initialize SPIFFS
  initSPIFFS();

  // Set the RTC using NTP
  setTheClock();
  clockResetTime = rtc.getEpoch() + threeDays + (unsigned long)(esp_random() >> 18);
  Serial.print("Clock will be set again at epoch time = ");
  Serial.println(clockResetTime);

  Serial.println(rtc.getEpoch());         //  (long)    1609459200 epoch without offset
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));   // (String) returns time with specified format 

  Serial.println("Initializing BME280...");
  initBME();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/current", HTTP_GET, sendCurrentReadings);

  // Request for chart of readings
  server.on("/chart.json", HTTP_GET, sendSensorLog);

  // Start server
  server.begin();

  Serial.println("Made it through setup()...");
}

void loop() {

  if (recordSample) {

    epochTime = rtc.getEpoch();

    timeStamps[sampleHeadPtr] = epochTime;
    //Serial.println(timeStamps[sampleHeadPtr]);

    dataPoints[sampleHeadPtr][0] = mySensorA.readTempF();;
    dataPoints[sampleHeadPtr][1] = mySensorA.readFloatHumidity();
    dataPoints[sampleHeadPtr][2] = mySensorA.dewPointF();
    dataPoints[sampleHeadPtr][3] = mySensorA.readFloatPressure();

    //Serial.println(sampleHeadPtr);

    sampleHeadPtr = (sampleHeadPtr + 1) % NSAMPLES;

    // if sampleHeadPtr equals 0 after incrementing then
    // we have come back to the beginning of the circular buffer storage array
    if (sampleHeadPtr == 0) {
      rollOver = true;
      Serial.println("Rollover occurred");
    }
    recordSample = false;

    // Reconnect to Wi-Fi if necessary
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to reconnect to Wi-Fi");
      while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        delay(3000);
      }
      Serial.println("\nRe-connected.");
    }

    // Set the clock using NTP about once every 74.25 hours (3 days + randomly up to 4.55 hours)
    if (epochTime > clockResetTime) {
      setTheClock();
      clockResetTime = epochTime + threeDays + (unsigned long)(esp_random() >> 18);
    }
  }
}

void sendCurrentReadings(AsyncWebServerRequest *request) {
  double tempF, relHum, dewPt, press;

  tempF = round(mySensorA.readTempF() * 10.0) / 10.0;
  relHum = round(mySensorA.readFloatHumidity());
  dewPt = round(mySensorA.dewPointF() * 10.0) / 10.0;
  press = mySensorA.readFloatPressure() / 100.0;

  // Create a plain text response with current readings
  String response = ("1348 Winfield Circle Garage Atmospheric Conditions<br>");
  response += "Temperature: " + String(tempF) + " &#8457<br>";
  response += "Humidity: " + String(relHum) + " %<br>";
  response += "Dew Point: " + String(dewPt) + " &#8457<br>";
  response += "Pressure: " + String(press) + " hPa<br>";

  // Send the plain text response
  request->send(200, "text/html", response);
}

void sendSensorLog(AsyncWebServerRequest *request) {
  AsyncWebServerResponse *response = request->beginChunkedResponse("application/json",
  // The following lambda function will be called each time the chunk transmitter is ready to
  // send a new chunk of the response.  This function will put data in 'buffer'.  The
  // 'index' variable tells us how many characters have been sent in the current response,
  // and starts at 0 when no characters have been sent yet.  Experiments have shown that
  // 'maxlen' is typically more than 5000 characters, but we'll keep it below 50% full.
  [](uint8_t *buffer, size_t maxlen, size_t index) -> size_t {
    static unsigned int sampleTailPtr;
    static boolean sentAllSamples;

    // Initialize the buffer as a null-terminated string
    buffer[0] = '\0';

    // If the caller tells us the index = 0, we're at the beginning of the chunked transmission
    if (index == 0) {

      // Flag to tell us when we're reading to send the JSON closing brackets
      sentAllSamples = false;

      // Start sending at the oldest sample
      if (!rollOver) {
        sampleTailPtr = 0;  // reset the log line index when we get a new request
      } else {
        sampleTailPtr = sampleHeadPtr;
      }

      // Copy the JSON document header to the buffer
      strcpy((char *)buffer, jsonHeader);
      // Now fall through and start sending samples, if there are any
    }

    // If we haven't sent all the samples yet...
    if (!sentAllSamples) {

      // The following line handles the case where the header is in the buffer
      size_t bufferLength = strlen((char *)buffer);

      // Fill the buffer a little over half full
      while (bufferLength < (maxlen / 2)) {

        // If there is a sample, build a JSON object containing an array row for the sample
        if (timeStamps[sampleTailPtr] > 0) {
          readings["unix_time"] = timeStamps[sampleTailPtr] * 1000UL;  // multiply by 1000 for Javascript time
          readings["tempF"] = round(dataPoints[sampleTailPtr][0] * 10.0) / 10.0;
          readings["relHum"] = round(dataPoints[sampleTailPtr][1] * 10.0) / 10.0;
          readings["dewPt"] = round(dataPoints[sampleTailPtr][2] * 10.0) / 10.0;
          readings["press"] = round(dataPoints[sampleTailPtr][3] * 10.0) / 1000.0;  // convert Pa to hPa
          serializeJson(readings, jsonString);

          // If there is at least one more sample to send, add a comma to separate JSON array rows
          if (((sampleTailPtr + 1) % NSAMPLES != sampleHeadPtr) && (timeStamps[sampleTailPtr] > 0)) {
            strcat(jsonString, ",");
          }

          // Write the JSON string to the chunk buffer and account for the increased buffer length
          snprintf((char *)buffer + bufferLength, maxlen, jsonString);
          bufferLength = strlen((char *)buffer);

          // Update the circular buffer tail pointer
          sampleTailPtr = (sampleTailPtr + 1) % NSAMPLES;
        }

        // Keep adding samples until the buffer level hits threshhold or we
        // run out of samples
        if ((sampleTailPtr == sampleHeadPtr) || (timeStamps[sampleTailPtr] == 0)) {
          break;
        }
      }

      // If we've sent all the samples, append the JSON closing brackets
      if ((sampleTailPtr == sampleHeadPtr) || (timeStamps[sampleTailPtr] == 0)) {
        strncat((char *)buffer + bufferLength, jsonFooter, maxlen);
        sentAllSamples = true;
      }

      // Let the caller know how many characters are in the buffer
      return strlen((char *)buffer);
    }

    // All the samples and the footer have been sent, so return 0 from the lambda function
    return 0;
  });

  request->send(response);
}
 


