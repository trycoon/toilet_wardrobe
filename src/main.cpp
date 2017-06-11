#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <AsyncMqttClient.h>
#include <main.h>

/*
 * Simple NodeMCU sketch that crossfades between two LEDs
 * depending on the status of a transmissive sensor.
 * This it used to visualize if the toilet i free or not.
 * The sketch also fades a additional LED depending on detection on motion from another sensor.
 */

uint8_t free_led_strength;
uint8_t occupied_led_strength;
boolean sensor_last_state;

boolean wardrobe_motion_detected;
uint8_t wardrobe_led_strength;
uint32_t wardrobe_delay;

AsyncMqttClient mqttClient;

void setup() {
  Serial.begin(115200);
  Serial.println("Toilet free indicator and wardrobe manager v1.1");

  pinMode(FREE_LED_PIN, OUTPUT);
  pinMode(OCCUPIED_LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  pinMode(WARDROBE_LED_PIN, OUTPUT);
  pinMode(WARDROBE_PIR_PIN, INPUT_PULLUP);

  // get initial status from sensor
  sensor_last_state = digitalRead(SENSOR_PIN);
  // set initial status on LEDs.
  if (sensor_last_state == HIGH) {
    set_occupied(false);
  } else {
    set_free(false);
  }

  update_led();

  setup_WiFi();
  setup_OTA();
  setup_MQTT();
}

void setup_WiFi() {
  WiFi.hostname(APP_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void setup_OTA() {
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  ArduinoOTA.setHostname(APP_NAME);

  // authentication string
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    publish_message("START UPDATING FIRMWARE");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    publish_message("DONE UPDATING FIRMWARE");
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
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_MQTT() {
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  //mqttClient.setCredentials("MQTT_USERNAME", "MQTT_PASSWORD");
  mqttClient.setKeepAlive(15); // seconds
  mqttClient.setClientId(APP_NAME);
  mqttClient.setWill(MQTT_TOPIC, 2, true, "DISCONNECTED");
  Serial.println("Connecting to MQTT broker...");
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

      char msg[] = "";
      strcat(msg, sensor_last_state ? "TOILET OCCUPIED": "TOILET FREE");
      publish_message(msg);
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
    publish_message("WARDROBE MOTION DETECTED");
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
  mqttClient.publish(MQTT_TOPIC, 1, true, "CONNECTED");
  Serial.println("Connected to the MQTT broker.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void publish_message(char* msg) {
  if (mqttClient.connected()) {
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC, 1, true, msg);
    Serial.println(packetIdPub1);
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.print("Disconnected from the MQTT broker! reason: ");
  Serial.println(static_cast<uint8_t>(reason));
  Serial.println("Reconnecting to MQTT...");
  mqttClient.connect();
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("MQTT Publish acknowledged");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
