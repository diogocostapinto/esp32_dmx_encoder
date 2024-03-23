#include <Arduino.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>
#include <WiFi.h>

// DMX and ArtNet Setup
const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 21;
const dmx_port_t dmxPort = 1;
ArtnetWifi artnet;
const int universe = 0;
byte data[DMX_PACKET_SIZE]; // DMX Data buffer
byte recordedData[DMX_PACKET_SIZE]; // Buffer for recording a single frame

// WiFi Configuration (Update with your details)
const char* ssid = "Quanta-Broadcast";
const char* password = "Quanta@2023!";
IPAddress artnetIp(10, 1, 9, 241);
IPAddress routerIP(10, 1, 8, 1);
IPAddress subnetIP(255, 255, 254, 0);

// Rotary Encoder Pins and State
const int encoderButtonPin = 27;
volatile bool buttonPressed = false;

// Mode Management
enum Mode { LIVE, RECORD, PLAYBACK };
volatile Mode currentMode = LIVE;

void IRAM_ATTR handleButtonPress() {
  // Debounce and ensure we act on press, not release
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) { // 200ms debounce period
    currentMode = static_cast<Mode>((currentMode + 1) % 3); // Cycle through the modes
    Serial.print("Switched to mode: ");
    switch (currentMode) {
      case LIVE:
        Serial.println("LIVE");
        break;
      case RECORD:
        Serial.println("RECORD");
        // Here you could reset any necessary variables for a new recording session
        break;
      case PLAYBACK:
        Serial.println("PLAYBACK");
        break;
    }
  }
  lastInterruptTime = interruptTime;
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  WiFi.config(artnetIp, routerIP, subnetIP);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void setupDMX() {
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmxPort, &config, NULL, 0);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
}

void setupArtNet() {
  artnet.begin();
  artnet.setArtDmxCallback([](uint16_t receivedUniverse, uint16_t length, uint8_t sequence, uint8_t* receivedData) {
    if (receivedUniverse == universe) {
      if (length <= DMX_PACKET_SIZE) {
        memcpy(&data[1], receivedData, length); // DMX data is 1-indexed
        dmx_write(dmxPort, data, length + 1);
        if (currentMode == LIVE) {
          Serial.println("ArtNet Data Received in LIVE Mode");
        } else if (currentMode == RECORD) {
          // Save the current frame (simple recording)
          memcpy(&recordedData[1], receivedData, length);
          Serial.println("Recorded one frame of DMX data");
          // After recording one frame, you might want to automatically switch to another mode
          // currentMode = PLAYBACK; // Optional: switch to playback or another mode
        }
      }
    }
  });
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  setupDMX();
  setupArtNet();

  pinMode(encoderButtonPin, INPUT_PULLUP); // Setup button pin
  attachInterrupt(digitalPinToInterrupt(encoderButtonPin), handleButtonPress, FALLING);
}

void loop() {
  artnet.read();
  if (currentMode == PLAYBACK) {
    // Example Playback logic for the recorded frame
    dmx_write(dmxPort, recordedData, sizeof(recordedData));
    Serial.println("Playing back recorded frame");
  }
 }
