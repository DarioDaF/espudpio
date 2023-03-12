
#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <EEPROM.h>

#include <polyfill.hpp>

#if defined(MODE_CLIENT) ^ defined(MODE_SERVER) == 0
  #error "Either MASTER or SLAVE must be specified"
#endif

// esp32 PINOUT
/*
0 = bootloader
1 = tx non usare
2 = bootloader + led
3 = rx non usare
4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 = libero
5 = booloader
6, 7, 8, 9, 10, 11 = flash (dipende dal modello di flash)
12 = bootloader
15 = bootloader
34, 35, 36, 37 = solo input

i2c = (21 = SDA, 22 = SCL)
uart0 = (1 = tx, 3 = rx)
uart2 = (16 = rx, 17 = tx)
spi = (19 = miso, 23 = mosi, 18 = sck, 2 = cs)
vspi = (19 = miso, 23 = mosi, 18 = sck, 5 = cs)
hspi = (12 = miso, 13 = mosi, 14 = sck, 15 = cs)
*/

// Used configs
#ifdef MODE_SERVER
  const int PIN_OUT[] =
  #ifdef ESP32
    { 33, 32, 4, 16, 17, 18, 19, 21, 22, 23 }
  #else
    { D0, D1, D2, D5, D6, D7 }
  #endif
  ;
#else
  #define VALUE_PIN_HIGH 0x00
  #define VALUE_PIN_LOW 0xFF
  const int PIN_IN1 = D1;
  unsigned long last_send;
#endif

#define LED_ON LOW
#define LED_OFF HIGH
#ifdef ESP32
  const int PIN_NET = 2;
#else
  const int PIN_NET = D4;
#endif

#define EE_MAGIC 0xDF01
#define DEF_SERVER_IP { 192, 168, 1, 50 }
struct __attribute__((packed)) MySettings {
  uint16_t magic = 0xDF01;
  char ssid[64] = "";
  char pass[64] = "";
  uint8_t server_ip[4] = DEF_SERVER_IP;
  uint16_t server_port = 5000;
  uint8_t local_ip[4] =
  #ifdef MODE_SERVER
    DEF_SERVER_IP
  #else
    { 0, 0, 0, 0 }
  #endif
  ;
  uint8_t gateway[4] = { 192, 168, 1, 1 };
  uint8_t mask[4] = { 255, 255, 255, 0 };

  // Client only
  unsigned long send_interval = 250UL;
  uint16_t target_idx = 0;
};
MySettings settings;
bool show_polling = false;
bool need_new_row = false;

void apply_new_row() {
  if(need_new_row) {
    Serial.println();
    need_new_row = false;
  }
}

void printSettings(bool showPass) {
  #ifdef MODE_SERVER
    Serial.println(F("[SERVER MODE]"));
  #else
    Serial.println(F("[CLIENT MODE]"));
  #endif
  Serial.print(F("Network Name (SSID): "));
  Serial.println(settings.ssid);

  Serial.print(F("Password: "));
  if (showPass) {
    Serial.println(settings.pass);
  } else {
    Serial.println(F("***"));
  }

  Serial.print(F("Server IP Address: "));
  Serial.println(IPAddress(settings.server_ip));

  Serial.print(F("Server Port: "));
  Serial.println(settings.server_port);

  Serial.print(F("Local IP Address: "));
  Serial.println(IPAddress(settings.local_ip));

  Serial.print(F("Subnet Mask: "));
  Serial.println(IPAddress(settings.mask));

  Serial.print(F("Gateway IP Address: "));
  Serial.println(IPAddress(settings.gateway));

  Serial.print(F("Send Interval: "));
  Serial.println(settings.send_interval);

  Serial.print(F("Target Index: "));
  Serial.println(settings.target_idx);
}

WiFiUDP udp;
bool WiFiConnected = false;

void reconnectWiFi() {
  IPAddress LOCAL_IP = settings.local_ip;
  if(lib::isSet(LOCAL_IP)) {
    WiFi.config(LOCAL_IP, settings.gateway, settings.mask);
  }
  WiFi.begin(settings.ssid, settings.pass);
}

void WiFiStationConnected(const lib::WiFiEventStationModeConnected& event) {
  apply_new_row();
  Serial.print(F("Connected to Access Point - WiFi Channel = "));
  Serial.print(event.channel);
  Serial.print(F(", Signal = "));
  Serial.print(WiFi.RSSI());
  Serial.println(F(" dbm"));
}

void WiFiGotIP(const lib::WiFiEventStationModeGotIP& event) {
  apply_new_row();
  WiFiConnected = true;
  Serial.print(F("Local IP Address: "));
  Serial.println(event.ip);
  Serial.print(F("Subnet Mask: "));
  Serial.println(event.mask);
  Serial.print(F("Gateway IP Address: "));
  Serial.println(event.gw);
  digitalWrite(PIN_NET, LED_ON);
}

void WiFiStationDisconnected(const lib::WiFiEventStationModeDisconnected& event) {
  apply_new_row();
  WiFiConnected = false;
  Serial.print(F("Disconnected from wifi for the following reason: Error -> "));
  Serial.println(event.reason);
  reconnectWiFi();
  digitalWrite(PIN_NET, LED_OFF);
}

lib::WiFiEventHandler hSMConnected, hSMGotIP, hSMDisconnected;

void setup() {
  WiFi.disconnect(true);
  pinMode(PIN_NET, OUTPUT);
  digitalWrite(PIN_NET, LED_OFF);
  Serial.begin(115200);
  delay(1000);
  #ifndef ESP32
    Serial.println();
  #endif
  Serial.println();
  EEPROM.begin(512);
  EEPROM.get(0, settings);
  if (settings.magic != EE_MAGIC) {
    // Invalid settings
    Serial.println(F("Invalid settings in EEPROM"));
    //settings = {};
    settings = MySettings();
  }
  printSettings(false);
  
  Serial.println();
  Serial.println(F("Wait for WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  hSMConnected = lib::onStationModeConnected(WiFi, WiFiStationConnected);
  hSMGotIP = lib::onStationModeGotIP(WiFi, WiFiGotIP);
  hSMDisconnected = lib::onStationModeDisconnected(WiFi, WiFiStationDisconnected);
  reconnectWiFi();

  #ifdef MODE_SERVER
    udp.begin(settings.server_port);
  #endif

  // Prepare pins
  #ifdef MODE_SERVER
    for(const int pin : PIN_OUT) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  #else
    pinMode(PIN_IN1, INPUT_PULLUP);
    last_send = millis();
  #endif
}

#define PKT_MAGIC 0xDF01
struct __attribute__((packed)) MyPkt {
  uint16_t magic;
  uint16_t idx;
  uint8_t value;
};

#ifdef MODE_SERVER
  char pkt_buff[200];

  void parsePacket() {
    size_t pkt_len = udp.parsePacket();
    
    if (pkt_len == 0)
      return; // No packet received

    if(pkt_len > sizeof(pkt_buff)) {
      // Packet too long
      apply_new_row();
      Serial.print(F("Received a packet too long: "));
      Serial.println(pkt_len);
      // If packet is not read parsePacket next will free the buffer anyway
      return;
    }
    
    // Read the packet
    udp.read(pkt_buff, pkt_len);
    IPAddress source_ip = udp.remoteIP();
    uint16_t source_port = udp.remotePort();

    if (pkt_len != sizeof(MyPkt)) {
      apply_new_row();
      Serial.print(F("Unknown packet length: "));
      Serial.println(pkt_len);
      return;
    }
    
    MyPkt* pkt = (MyPkt*)pkt_buff;
    if (pkt->magic != PKT_MAGIC) {
      apply_new_row();
      Serial.print(F("Invalid version: 0x"));
      Serial.println(pkt->magic, HEX);
      return;
    }

    if (pkt->idx >= lib::size(PIN_OUT)) {
      apply_new_row();
      Serial.print(F("Invalid target: "));
      Serial.println(pkt->idx);
      return;
    }

    bool prev_state = digitalRead(PIN_OUT[pkt->idx]);
    digitalWrite(PIN_OUT[pkt->idx], pkt->value ? HIGH : LOW);
    bool new_state = digitalRead(PIN_OUT[pkt->idx]);
    if (prev_state != new_state) {
      apply_new_row();
      Serial.print(F("Changed value of output "));
      Serial.print(pkt->idx);
      if (new_state)
        Serial.print(F(" to HIGH"));
      else
        Serial.print(F(" to LOW"));
      Serial.print(F("; command sent by "));
      Serial.print(source_ip);
      Serial.print(F(":"));
      Serial.println(source_port);
    } else {
      if (show_polling) {
        Serial.print(pkt->idx, HEX); //Valido per max 16 elementi 0..f
        need_new_row = true;
      }
    }
  }
#endif

#ifdef MODE_CLIENT
  uint8_t old_state = VALUE_PIN_HIGH;
#endif

String serialReadLine() {
  String res = "";
  while (true) {
    if (Serial.available()) {
      int c = Serial.read();
      Serial.print((char)c);
      if (c == '\n') {
        return res;
      } else if(c != '\r') {
        res += (char)c;
      }
    }
  }
}

void serialReadIP(uint8_t ip[4]) {
  IPAddress ip_obj;
  ip_obj.fromString(serialReadLine());
  const auto v4 = lib::v4(ip_obj);
  memcpy(ip, &v4, 4);
}

void readSettings() {
  String s;
  IPAddress ip;

  MySettings _def;

  memset((void*)&settings, 0, sizeof(MySettings));
  settings.magic = _def.magic;
  Serial.println();
  Serial.print(F("Network Name (SSID): "));
  serialReadLine().toCharArray(settings.ssid, sizeof(settings.ssid) - 1);

  Serial.print(F("Password: "));
  serialReadLine().toCharArray(settings.pass, sizeof(settings.pass) - 1);

  Serial.print(F("Server IP Address: "));
  serialReadIP(settings.server_ip);

  Serial.print(F("Server Port: "));
  settings.server_port = serialReadLine().toInt();

  #ifdef MODE_SERVER
    memcpy(settings.local_ip, settings.server_ip, 4);
  #else
    Serial.print(F("Local IP Address: "));
    serialReadIP(settings.local_ip);
  #endif

  if (lib::isSet(IPAddress(settings.local_ip))) {
    Serial.print(F("Subnet Mask: "));
    serialReadIP(settings.mask);

    Serial.print(F("Gateway IP Address: "));
    serialReadIP(settings.gateway);
  }

  #ifdef MODE_CLIENT
    Serial.print(F("Send Interval: "));
    settings.send_interval = serialReadLine().toInt();

    Serial.print(F("Target Index: "));
    settings.target_idx = serialReadLine().toInt();
  #else
    settings.send_interval = _def.send_interval;
    settings.target_idx = _def.target_idx;
  #endif

  Serial.println();
  printSettings(true);
  Serial.print("Save? [N/y] ");
  s = serialReadLine();
  s.toUpperCase();
  if (s == "Y") {
    EEPROM.put(0, settings);
    EEPROM.commit();
    Serial.println(F("SAVED!"));
  }
}

[[noreturn]] void askSettings() {
  Serial.println();
  readSettings();
  Serial.println(F("REBOOT required!"));
  while(true);
}

void loop() {
  yield();
  if (Serial.available()) {
    char value = Serial.read();
    if (value == '!') {
      udp.stop();
      apply_new_row();
      askSettings(); // noreturn
    } else if (value == '?') {
      show_polling = !show_polling;
    } else if (value == '\\') {
      apply_new_row();
      Serial.print(F("WiFi Signal = "));
      Serial.print(WiFi.RSSI());
      Serial.println(F(" dbm"));
    }
  }
  if(WiFiConnected) {
    #ifdef MODE_SERVER
      parsePacket();
    #else
      auto current_time = millis();
      if (current_time - last_send > settings.send_interval) {
        MyPkt pkt;
        memset(&pkt, 0, sizeof(MyPkt));
        pkt.magic = settings.magic;
        pkt.idx = settings.target_idx;
        pkt.value = digitalRead(PIN_IN1) ? VALUE_PIN_HIGH : VALUE_PIN_LOW;
        if (pkt.value != old_state) {
          apply_new_row();
          Serial.println("Changed STATE!");
          old_state = pkt.value;
        } 
        udp.beginPacket(settings.server_ip, settings.server_port);
        udp.write((char*)&pkt, sizeof(MyPkt));
        udp.endPacket();
        if (show_polling) {
          Serial.print('.');
          need_new_row = true;
        }
        last_send = current_time;
      }
    #endif
  }
}
