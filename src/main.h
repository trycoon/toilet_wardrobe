#ifndef main_h
#define main_h

#include <Arduino.h>
#include <AsyncMqttClient.h>

const uint8_t FREE_LED_PIN = D1;
const uint8_t OCCUPIED_LED_PIN = D2;
const uint8_t SENSOR_PIN = D3;   // HIGH=occupied.
const uint8_t BRIGHTNESS = 150;
const uint8_t WARDROBE_PIR_PIN = D5;
const uint8_t WARDROBE_LED_PIN = D6;
const uint32_t WARDROBE_ON_DELAY = 10000;
const char* APP_NAME = "toilet-wardrobe";

const char* SSID = "<your SSID>";
const char* WIFI_PASSWORD = "<your wifi password>";

const char* OTA_PASSWORD = "<a OTA password";

const char* MQTT_SERVER = "<your MQTT server IP>";
const uint16_t MQTT_PORT = 1883;
const char MQTT_TOPIC[] = "home/toilet-wardrobe";

// https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
// The number of Steps between the output being on and off
const uint8_t OCCUPIED_PWM_INTERVALS = 70;
const float OCCUPIED_FADE_RATIO = (OCCUPIED_PWM_INTERVALS * log10(2))/(log10(BRIGHTNESS));
const uint8_t WARDROOBE_PWM_INTERVALS = 70;
const float WARDROBE_FADE_RATIO = (WARDROOBE_PWM_INTERVALS * log10(2))/(log10(255));

void set_free(boolean animate);
void set_occupied(boolean animate);
void update_led();
void update_wardrobe_fade();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void publish_message(char* msg);
void setup_WiFi();
void setup_OTA();
void setup_MQTT();

#endif
