#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncMqttClient.h>

/*
 * Simple Arduino sketch that crossfades between two LEDs
 * depending on the status of a transmissive sensor.
 * This it used to visualize if the toilet i free or not.
 * The sketch also fades a additional LED depending on detection on motion from another sensor.
 */

const uint8_t FREE_LED_PIN = 6;
const uint8_t OCCUPIED_LED_PIN = 5;
const uint8_t SENSOR_PIN = 4;   // HIGH=occupied.
const uint8_t BRIGHTNESS = 150;
const uint8_t WARDROBE_PIR_PIN = 8;
const uint8_t WARDROBE_LED_PIN = 9;
const uint32_t WARDROBE_ON_DELAY = 10000;
const char* APP_NAME = "toilet-wardrobe";

const char* SSID = "";
const char* WIFI_PASSWORD = "";

const char* OTA_PASSWORD = "";

const char* MQTT_USERNAME = APP_NAME;
const char* MQTT_PASSWORD = "";
const IPAddress MQTT_SERVER = (192, 168, 10, 110);
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC = "home/" + APP_NAME;

// https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
// The number of Steps between the output being on and off
const uint8_t OCCUPIED_PWM_INTERVALS = 50;
const float OCCUPIED_FADE_RATIO = (OCCUPIED_PWM_INTERVALS * log10(2))/(log10(BRIGHTNESS));
const uint8_t WARDROOBE_PWM_INTERVALS = 50;
const float WARDROBE_FADE_RATIO = (WARDROOBE_PWM_INTERVALS * log10(2))/(log10(255));

uint8_t free_led_strength;
uint8_t occupied_led_strength;
boolean sensor_last_state;

boolean wardrobe_motion_detected;
uint8_t wardrobe_led_strength;
uint32_t wardrobe_delay;

AsyncMqttClient mqttClient;

// function declarations
void set_free(boolean animate);
void set_occupied(boolean animate);
void update_led();
void update_wardrobe_fade();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void publish_message(char* msg);


void setup() {
  Serial.begin(115200);
  Serial.println("Toilet free indicator and wardrobe manager v1.0");

  pinMode(FREE_LED_PIN, OUTPUT);
  pinMode(OCCUPIED_LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);

  pinMode(WARDROBE_LED_PIN, OUTPUT);
  pinMode(WARDROBE_PIR_PIN, INPUT);

  // get initial status from sensor
  sensor_last_state = digitalRead(SENSOR_PIN);
  // set initial status on LEDs.
  if (sensor_last_state == HIGH) {
    set_occupied(false);
  } else {
    set_free(false);
  }

  update_led();

  // connect to WiFi accesspoint
  WiFi.hostname(APP_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(APP_NAME);

  // authentication string
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    publish_message("Start updating firmare.");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    publish_message("Done updating firmare.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // MQTT stuff
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.setClientId(APP_NAME);
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void loop() {
  ArduinoOTA.handle();

  // check if sensor has changed since last?
  if (digitalRead(SENSOR_PIN) != sensor_last_state) {
    // debounce and nice crossfade effect delay, for better effect. :-)
    delay(1000);
    // check if state is still changed, debounce.
    if (digitalRead(SENSOR_PIN) != sensor_last_state) {
      delay(1500);
      sensor_last_state = !sensor_last_state;
      String resultString = "Toilet occupied=";
      resultString += sensor_last_state ? "true": "false";
      Serial.println(resultString);

      publish_message("Toilet occupied " + (sensor_last_state ? "true": "false"));
    }
  }

  if (sensor_last_state == HIGH) {
    set_occupied(true);
  } else {
    set_free(true);
  }

  update_led();

  if (digitalRead(WARDROBE_PIR_PIN) == HIGH && wardrobe_motion_detected == false) {
    Serial.println("Wardrobe motion detected.");
    publish_message("Wardrobe motion detected");
    wardrobe_motion_detected = true;
  }

  update_wardrobe_fade();

  // every loop is 10 milliseconds to make a visual fade.
  delay(10);
}

void set_free(boolean animate) {
  if (animate) {
    // make free_led a bit brigther for every loop until we hit max level.
    if (free_led_strength < OCCUPIED_PWM_INTERVALS) {
      free_led_strength++;
    }
    // make occupied_led a bit duller for every loop until we hit min level.
    if (occupied_led_strength > 0) {
      occupied_led_strength--;
    }
  } else {
    free_led_strength = BRIGHTNESS;
    occupied_led_strength = 0;
  }
}

void set_occupied(boolean animate) {
  if (animate) {
    // make occupied_led a bit brigther for every loop until we hit max level.
    if (occupied_led_strength < OCCUPIED_PWM_INTERVALS) {
      occupied_led_strength++;
    }
    // make free_led a bit duller for every loop until we hit min level.
    if (free_led_strength > 0) {
      free_led_strength--;
    }
  } else {
    free_led_strength = 0;
    occupied_led_strength = BRIGHTNESS;
  }
}

void update_led() {
  analogWrite(FREE_LED_PIN, 255 - (pow (2, (free_led_strength / OCCUPIED_FADE_RATIO)) - 1));           // invert values, 255=full bright light, 0=light off.
  analogWrite(OCCUPIED_LED_PIN, 255 - (pow (2, (occupied_led_strength / OCCUPIED_FADE_RATIO)) - 1));   // invert values, 255=full bright light, 0=light off.
}

void update_wardrobe_fade() {
  if (wardrobe_motion_detected == true) {
    // if LED-stripe has not reached max intensity yet, keep make it brigther.
    if (wardrobe_led_strength < WARDROOBE_PWM_INTERVALS) {
      wardrobe_led_strength++;
    } else {
      // if we have reached max brightness, keep it there for a defined time/delay.
      if (wardrobe_delay >= WARDROBE_ON_DELAY) {
        wardrobe_delay = 0;
        wardrobe_motion_detected = false;
      } else {
        wardrobe_delay++;
      }
    }
  } else {
    // no motion has been detected, start making LED-stripe dimmer.
    if (wardrobe_led_strength > 0) {
      wardrobe_led_strength--;
    }
  }

  analogWrite(WARDROBE_LED_PIN, 255 - (pow (2, (wardrobe_led_strength / WARDROBE_FADE_RATIO)) - 1));       // invert values, 255=full bright light, 0=light off.
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to the MQTT broker.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  publish_message(APP_NAME + " CONNECTED");
}

void publish_message(char const * msg) {
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC, 1, true, msg);
  Serial.println(packetIdPub1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from the MQTT broker!");
  Serial.println("Reconnecting to MQTT...");
  mqttClient.connect();
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("MQTT Publish acknowledged");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
