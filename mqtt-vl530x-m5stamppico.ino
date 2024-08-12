#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "config.h"

WiFiClient wifi_client;
PubSubClient mqtt_client(mqtt_host, mqtt_port, NULL, wifi_client);

// see also...https://github.com/m5stack/M5Stack/blob/master/examples/Unit/ToF_VL53L0X/ToF_VL53L0X.ino
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define address 0x29  // I2C address

byte gbuf[16];

uint16_t bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}

uint16_t makeuint16(int lsb, int msb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

byte read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

byte read_byte_data_at(byte reg) {
  // write_byte_data((byte)0x00);
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

uint16_t read_word_data_at(byte reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf);
}

void read_block_data_at(byte reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i = 0; i < sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}

uint16_t read_distance() {
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt = 0;
  while (cnt < 100) {  // 1 second waiting time max
    delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
  if (val & 0x01) {
    // nothing to do...
  } else {
    // not ready
    return 0xffff;
  }

  read_block_data_at(VL53L0X_REG_RESULT_RANGE_STATUS, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

  return dist;
}

#define RGB_LED_PIN 27  // M5Stamp pico
#define RGB_LED_NUM 1
Adafruit_NeoPixel rgb_led(RGB_LED_NUM, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
  rgb_led.setPixelColor(0, rgb_led.Color(r, g, b));
  rgb_led.show();
}

void reboot() {
  Serial.println("REBOOT!!!!!");
  for (int i = 0; i < 30; ++i) {
    set_rgb_led(255, 0, 255);
    delay(100);
    set_rgb_led(0, 0, 0);
    delay(100);
  }

  ESP.restart();
}

void setup() {
  Wire.begin(21, 22);
  Wire.setClock(20000); 

  Serial.begin(9600);

  // VL53L0Xへの電源供給用
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);

  // Wifi
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  WiFi.setSleep(false);
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    switch (count % 4) {
      case 0:
        Serial.println("|");
        set_rgb_led(30, 30, 0);
        break;
      case 1:
        Serial.println("/");
        break;
      case 2:
        set_rgb_led(0, 0, 0);
        Serial.println("-");
        break;
      case 3:
        Serial.println("\\");
        break;
    }
    count++;
    if (count >= 240) reboot();  // 240 / 4 = 60sec
  }
  set_rgb_led(30, 30, 0);
  Serial.println("WiFi connected!");

  // MQTT
  bool rv = false;
  if (mqtt_use_auth == true) {
    rv = mqtt_client.connect(mqtt_client_id, mqtt_username, mqtt_password);
  } else {
    rv = mqtt_client.connect(mqtt_client_id);
  }
  if (rv == false) {
    Serial.println("mqtt connecting failed...");
    reboot();
  }
  Serial.println("MQTT connected!");
  set_rgb_led(0, 0, 30);

  // configTime
  configTime(9 * 3600, 0, "ntp.nict.jp");
  struct tm t;
  if (!getLocalTime(&t)) {
    Serial.println("getLocalTime() failed...");
    delay(500);
    reboot();
  }
  Serial.println("configTime() success!");
  set_rgb_led(0, 30, 0);

  publish_distance();
}

#define SAMPLE_COUNT 10

void publish_distance() {
  // N回測定して平均を求める
  long total_dist = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    total_dist += read_distance();
    delay(10);
  }

  int avg_dist = total_dist / SAMPLE_COUNT;

  // G36で電源の電圧を測定
  float voltage = analogReadMilliVolts(36) / 1000.0f;
 
  // publishするメッセージの組み立て
  char buf[256];
  
  time_t t;
  struct tm *tm;

  t = time(NULL);
  tm = localtime(&t);

  snprintf(buf, 255, "{\"time\":\"%4d-%02d-%02dT%02d:%02d:%02d\", \"voltage\", %.3f, \"distance\":%u}", 
    tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
    tm->tm_hour, tm->tm_min, tm->tm_sec,
    voltage,
    avg_dist);

  // メッセージをpublish (retain有効)
  Serial.println(buf);
  mqtt_client.publish(mqtt_publish_topic, buf, true);

  delay(30);

  set_rgb_led(0, 0, 0);
  WiFi.disconnect(true);
  ESP.deepSleep(60 * 1000 * 1000UL);
}

void loop() {
}
