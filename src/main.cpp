
#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <EEPROM.h>

// For std::size
#include <array>


#if defined(MODE_CLIENT) ^ defined(MODE_SERVER) == 0
  #error "Either MASTER or SLAVE must be specified"
#endif

// Used configs
#ifdef MODE_SERVER
  const int PIN_OUT[] =
  #ifdef ESP32
    { 34, 35, 32, 33, 25, 26, 27, 14, 12, 13 }
  #else
    { D0, D1, D2, D5, D6, D7 }
  #endif
  ;
#else
  #define PIN_ON 0x00
  #define PIN_OFF 0xFF
  const int PIN_IN1 = D1;

  unsigned long last_send;
#endif

#define EE_MAGIC 0xDF01
#define DEF_SERVER_IP { 192, 168, 1, 50 }
struct __attribute__((packed)) MySettings {
  uint16_t magic = 0xDF01;
  char ssid[64] = "";
  char pass[64] = "";
  uint8_t server_ip[4] = DEF_SERVER_IP;
  uint16_t server_port = 5000;
  uint8_t my_ip[4] =
  #ifdef MODE_SERVER
    DEF_SERVER_IP
  #else
    { 0, 0, 0, 0 }
  #endif
  ;
  uint8_t gateway[4] = { 192, 168, 1, 1 };
  uint8_t mask[4] = { 255, 255, 255, 0 };

  // Client only
  unsigned long send_interval = 100UL;
  uint16_t target_idx = 0;
};
MySettings settings;
bool show_received = false;

void printSettings(bool showPass) {
  Serial.println();
  #ifdef MODE_SERVER
    Serial.println(F("[SERVER MODE]"));
  #else
    Serial.println(F("[CLIENT MODE]"));
  #endif
  Serial.print(F("SSID: "));
  Serial.println(settings.ssid);

  Serial.print(F("PASS: "));
  if (showPass) {
    Serial.println(settings.pass);
  } else {
    Serial.println(F("***"));
  }

  Serial.print(F("Server IP: "));
  Serial.println(IPAddress(settings.server_ip));

  Serial.print(F("Server Port: "));
  Serial.println(settings.server_port);

  Serial.print(F("My IP: "));
  Serial.println(IPAddress(settings.my_ip));

  Serial.print(F("Gateway: "));
  Serial.println(IPAddress(settings.gateway));

  Serial.print(F("Netmask: "));
  Serial.println(IPAddress(settings.mask));

  Serial.print(F("Send Interval: "));
  Serial.println(settings.send_interval);

  Serial.print(F("Target Index: "));
  Serial.println(settings.target_idx);
}

WiFiUDP udp;
bool WiFiConnected = false;

void reconnectWiFi() {
  IPAddress MY_IP = settings.my_ip;
  if(MY_IP.isSet()) {
    WiFi.config(MY_IP, settings.gateway, settings.mask);
  }
  WiFi.begin(settings.ssid, settings.pass);
}

void WiFiStationConnected(const WiFiEventStationModeConnected& event){
  Serial.println(F("Connected to AP successfully!"));
  Serial.print(F("AP signal: "));
  Serial.print(WiFi.RSSI());
  Serial.println(F(" dbm"));
}

void WiFiGotIP(const WiFiEventStationModeGotIP& event){
  WiFiConnected = true;
  Serial.print(F("Local IP address: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("Subnet Mask: "));
  Serial.println(WiFi.subnetMask());
  Serial.print(F("Gateway IP address: "));
  Serial.println(WiFi.gatewayIP());
}

void WiFiStationDisconnected(const WiFiEventStationModeDisconnected& event){
  WiFiConnected = false;
  Serial.println(F("Disconnected from WiFi access point"));
  reconnectWiFi();
}

WiFiEventHandler hSMConnected, hSMGotIP, hSMDisconnected;

void setup() {
  WiFi.disconnect(true);
  delay(2000);
  Serial.begin(115200);
  Serial.println();
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
  hSMConnected = WiFi.onStationModeConnected(WiFiStationConnected);
  hSMGotIP = WiFi.onStationModeGotIP(WiFiGotIP);
  hSMDisconnected = WiFi.onStationModeDisconnected(WiFiStationDisconnected);
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
      Serial.print(F("Unknown packet length: "));
      Serial.println(pkt_len);
      return;
    }
    
    MyPkt* pkt = (MyPkt*)pkt_buff;
    if (pkt->magic != PKT_MAGIC) {
      Serial.print(F("Invalid version: "));
      Serial.println(pkt->magic, HEX);
      return;
    }

    if (pkt->idx >= std::size(PIN_OUT)) {
      Serial.print(F("Invalid target: "));
      Serial.println(pkt->idx);
      return;
    }

    bool prev_state = digitalRead(PIN_OUT[pkt->idx]);
    digitalWrite(PIN_OUT[pkt->idx], pkt->value ? HIGH : LOW);
    bool new_state = digitalRead(PIN_OUT[pkt->idx]);
    if (prev_state != new_state) {
      Serial.println();
      Serial.print(F("Change state for "));
      Serial.print(pkt->idx);
      if (new_state)
        Serial.print(F(" to HIGH from "));
      else
        Serial.print(F(" to LOW from "));
      Serial.print(source_ip);
      Serial.print(F(":"));
      Serial.println(source_port);
    } else {
      if (show_received)
        Serial.print(pkt->idx);
    }
  }
#endif

#ifdef MODE_CLIENT
  uint8_t old_state = true;
#endif

bool in_settings = false;

String serialReadLine() {
  String res = "";
  while (true) {
    if (Serial.available()) {
      int c = Serial.read();
      if (c == '\n') {
        Serial.print((char)c);
        return res;
      } else if(c== '\r') {
        Serial.print((char)c);
      } else {
        res += (char)c;
        Serial.print((char)c);
      }
    }
  }
}

void serialReadIP(uint8_t ip[4]) {
  IPAddress ip_obj;
  ip_obj.fromString(serialReadLine());
  memcpy(ip, &ip_obj.v4(), 4);
}

void readSettings() {
  String s;
  IPAddress ip;

  MySettings _def;

  memset((void*)&settings, 0, sizeof(MySettings));
  settings.magic = _def.magic;
  Serial.println();
  Serial.print(F("SSID: "));
  serialReadLine().toCharArray(settings.ssid, sizeof(settings.ssid) - 1);

  Serial.print(F("PASS: "));
  serialReadLine().toCharArray(settings.pass, sizeof(settings.pass) - 1);

  Serial.print(F("Server IP: "));
  serialReadIP(settings.server_ip);

  Serial.print(F("Server Port: "));
  settings.server_port = serialReadLine().toInt();

  #ifdef MODE_SERVER
    memcpy(settings.my_ip, settings.server_ip, 4);
  #else
    Serial.print(F("My IP: "));
    serialReadIP(settings.my_ip);
  #endif

  if (IPAddress(settings.my_ip).isSet()) {
    Serial.print(F("Gateway: "));
    serialReadIP(settings.gateway);

    Serial.print(F("Netmask: "));
    serialReadIP(settings.mask);
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

void loop() {
  yield();
  if(in_settings) {
    Serial.println();
    readSettings();
    Serial.println(F("REBOOT required!"));
    while(true);
  } else if(WiFiConnected) {
    if (Serial.available()) {
      char value = Serial.read();
      if (value == '!') {
        udp.stop();
        in_settings = true;
        return;
      }
      if (value == '?') {
        show_received = !show_received;
        return;
      }

    }
    #ifdef MODE_SERVER
      parsePacket();
    #else
      auto current_time = millis();
      if (current_time - last_send > settings.send_interval) {
        Serial.print('.');
        MyPkt pkt;
        memset(&pkt, 0, sizeof(MyPkt));
        pkt.magic = PKT_MAGIC;
        pkt.idx = settings.target_idx;
        pkt.value = digitalRead(PIN_IN1) ? PIN_ON : PIN_OFF;
        if (pkt.value != old_state) {
          Serial.println();
          Serial.println("Changed STATE!");
          old_state = pkt.value;
        }
        udp.beginPacket(settings.server_ip, settings.server_port);
        udp.write((char*)&pkt, sizeof(MyPkt));
        udp.endPacket();

        last_send = current_time;
      }
    #endif
  }
}
