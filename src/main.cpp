
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

using millis_t = unsigned long;

struct IPPort : public Printable {
  
  IPAddress ip;
  uint16_t port;

  operator uint64_t() const {
    uint64_t res = (uint32_t)ip;
    res <<= 32;
    res |= port;
    return res;
  }

  IPPort(IPAddress ip, uint16_t port) : ip(ip), port(port) {}
  IPPort() : ip(INADDR_NONE), port(0) {}

  virtual size_t printTo(Print& p) const {
    size_t res = p.print(ip);
    res += p.print(':');
    res += p.print(port);
    return res;
  }
};

#ifdef MODE_SERVER

  void apply_new_row();

  #include <map>

  std::map<IPPort, millis_t> track_last_received;

  bool track_received(const IPPort& ipp, millis_t time) {
    bool notPresent = track_last_received.find(ipp) == track_last_received.end();
    track_last_received[ipp] = time;
    if (notPresent) {
      apply_new_row();
      // Connected
      Serial.print(F("Connected client: "));
      Serial.println(ipp);
    }
    return notPresent;
  }

  int track_older(millis_t time, unsigned int expire) {
    // Taken from https://en.cppreference.com/w/cpp/container/map/erase_if
    int expire_count = 0;
    for (auto it = track_last_received.begin(), last = track_last_received.end(); it != last; ) {
      if (time - it->second > expire) { // Expire condition
        expire_count += 1;
        apply_new_row();
        // Disconnected
        Serial.print(F("Disconnected client: "));
        Serial.println(it->first);

        it = track_last_received.erase(it);
      } else {
        ++it;
      }
    }
    return expire_count;
  }

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
    //{ 4, 16, 17, 18, 19, 23, 13, 27, 26, 25, 33, 32 }
    { 33, 32, 4, 16, 17, 18, 19, 21, 22, 23 }
  #else
    { D0, D1, D2, D5, D6, D7 }
  #endif
  ;
#else
  #define VALUE_PIN_HIGH 0x00
  #define VALUE_PIN_LOW 0xFF
  #define SWICH_OPEN VALUE_PIN_HIGH
  #define SWICH_CLOSED VALUE_PIN_LOW
  #ifdef ESP32
    const int PIN_IN1 = 13;
  #else
    const int PIN_IN1 = D1;
  #endif
  millis_t last_send;
#endif

#define LED_ON LOW
#define LED_OFF HIGH
#ifdef ESP32
  const int PIN_NET = 2;
#else
  const int PIN_NET = D4;
#endif

#include <defSettings.hpp>
#define EE_MAGIC 0xDF01
struct __attribute__((packed)) settings_s {
  uint16_t magic = EE_MAGIC;
  char ssid[64] = DEFSETTINGS_SSID;
  char pass[64] = DEFSETTINGS_PASS;
  uint8_t server_ip[4] = DEFSETTINGS_SERVER_IP;
  uint16_t server_port = 5000;
  uint8_t local_ip[4] =
  #ifdef MODE_SERVER
    DEFSETTINGS_SERVER_IP
  #else
    { 0, 0, 0, 0 }
  #endif
  ;
  uint8_t gateway[4] = DEFSETTINGS_GATEWAY_IP;
  uint8_t mask[4] = DEFSETTINGS_MASK_IP;

  unsigned long send_interval = 
  #ifdef MODE_SERVER
    1000UL
  #else
    100UL
  #endif
  ;
  // Client only
  uint16_t target_idx = 0;
};

settings_s settings;
bool show_polling = false;
bool need_new_row = false;

void apply_new_row() {
  if(need_new_row) {
    Serial.println();
    need_new_row = false;
  }
}

void printSettings(Print& out, bool showPass) {
  #ifdef MODE_SERVER
    out.println(F("[SERVER MODE]"));
  #else
    out.println(F("[CLIENT MODE]"));
  #endif
  out.print(F("Network Name (SSID): "));
  out.println(settings.ssid);

  out.print(F("Password: "));
  if (showPass) {
    out.println(settings.pass);
  } else {
    out.println(F("***"));
  }

  out.print(F("Server IP Address: "));
  out.println(IPAddress(settings.server_ip));

  out.print(F("Server Port: "));
  out.println(settings.server_port);

  out.print(F("Local IP Address: "));
  out.println(IPAddress(settings.local_ip));

  out.print(F("Subnet Mask: "));
  out.println(IPAddress(settings.mask));

  out.print(F("Gateway IP Address: "));
  out.println(IPAddress(settings.gateway));

  #ifdef MODE_SERVER
    out.print(F("Expire Interval: "));
  #else
    out.print(F("Send Interval: "));
  #endif
  out.println(settings.send_interval);

  #ifndef MODE_SERVER
    out.print(F("Target Index: "));
    out.println(settings.target_idx);
  #endif
}

WiFiUDP udp;
bool WiFiConnected = false;

[[noreturn]] void askSettings();

void reconnectWiFi() {
  if (settings.ssid[0] == '\0') {
    Serial.println(F("Empty SSID, reenter settings"));
    askSettings();
  }
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

void disconnectWiFi() {
  udp.stop();
  hSMConnected = nullptr;
  hSMGotIP = nullptr;
  hSMDisconnected = nullptr;
  WiFi.disconnect(true);
  WiFiConnected = false;
}

void setup() {
  disconnectWiFi();
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
    settings = settings_s();
  }
  printSettings(Serial, false);
  
  Serial.println();
  Serial.println(F("Wait for WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  hSMConnected = lib::onStationModeConnected(WiFi, WiFiStationConnected);
  hSMGotIP = lib::onStationModeGotIP(WiFi, WiFiGotIP);
  hSMDisconnected = lib::onStationModeDisconnected(WiFi, WiFiStationDisconnected);
  reconnectWiFi();

  udp.begin(settings.server_port); // Server for both for general info

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
struct __attribute__((packed)) pkt_s {
  uint16_t magic;
  uint16_t idx;
  uint8_t value;
};

#define MAX_UDP_PACKET 1024
size_t pkt_len = 0;
char pkt_buff[MAX_UDP_PACKET];
IPPort pkt_source;

bool execQuery(char query, Print& out);
bool parseQueryPacket() {
  if (pkt_len != 2)
    return false;
  if (pkt_buff[0] != '?')
    return false;

  char query = pkt_buff[1];

  lib::PrintBuff pbuff(sizeof(pkt_buff), pkt_buff);
  if (!execQuery(query, pbuff)) {
    pbuff.print(F("Unknown query: "));
    pbuff.println(query);
    // Accept them anyway cause they are "valid" but unknown!!!
  }

  udp.beginPacket(pkt_source.ip, pkt_source.port);
  lib::writeRaw(udp, pbuff.getBuff(), pbuff.getPos());
  udp.endPacket();

  return true;
}

#ifdef MODE_SERVER
  bool parsePktPacket() {
    if (pkt_len != sizeof(pkt_s)) {
      apply_new_row();
      Serial.print(F("Unknown packet length: "));
      Serial.println(pkt_len);
      return false;
    }
    
    pkt_s* pkt = (pkt_s*)pkt_buff;
    if (pkt->magic != PKT_MAGIC) {
      apply_new_row();
      Serial.print(F("Invalid version: 0x"));
      Serial.println(pkt->magic, HEX);
      return false;
    }

    if (pkt->idx >= lib::size(PIN_OUT)) {
      apply_new_row();
      Serial.print(F("Invalid target: "));
      Serial.println(pkt->idx);
      return false;
    }

    track_received(pkt_source, millis());

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
      Serial.println(pkt_source);
    } else {
      if (show_polling) {
        Serial.print(pkt->idx, HEX); //Valido per max 16 elementi 0..f
        need_new_row = true;
      }
    }

    return true;
  }
#endif

void parsePacket() {
  pkt_len = udp.parsePacket();

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
  pkt_source.ip = udp.remoteIP();
  pkt_source.port = udp.remotePort();
  udp.read(pkt_buff, pkt_len);

  bool parsed = false;

  if (!parsed)
    parsed = parseQueryPacket();

  #ifdef MODE_SERVER
    if (!parsed)
      parsed = parsePktPacket();
  #endif

  pkt_len = 0;
}

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

  settings_s _def;

  memset((void*)&settings, 0, sizeof(settings_s));
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

  #ifdef MODE_SERVER
    Serial.print(F("Expire Interval: "));
  #else
    Serial.print(F("Send Interval: "));
  #endif
  settings.send_interval = serialReadLine().toInt();

  #ifdef MODE_CLIENT  
    Serial.print(F("Target Index: "));
    settings.target_idx = serialReadLine().toInt();
  #else
    settings.target_idx = _def.target_idx;
  #endif

  Serial.println();
  printSettings(Serial, true);
  Serial.print(F("Save? [N/y]: "));
  s = serialReadLine();
  s.toUpperCase();
  if (s == "Y") {
    EEPROM.put(0, settings);
    EEPROM.commit();
    Serial.println(F("SAVED!"));
  }
}

[[noreturn]] void reboot() {
  disconnectWiFi();
  Serial.println(F("Wait for Reboot..."));
  Serial.println();
  Serial.flush();
  ESP.restart();
  while(true);
}

[[noreturn]] void askSettings() {
  Serial.println();
  readSettings();
  reboot();
}

bool execQuery(char query, Print& out) {
  if ((query == 'I') || (query == 'i')) {
    printSettings(out, false);
    return true;
  } else if ((query == 'W') || (query == 'w')) {
    if(WiFi.isConnected()) {
      out.print(F("WiFi Signal = "));
      out.print(WiFi.RSSI());
      out.println(F(" dbm"));
    } else {
      out.println(F("WiFi Signal = <NOT CONNECTED>"));
    }
    return true;
  }
  #ifdef MODE_SERVER
    else if ((query == 'C') || (query == 'c')) {
      out.println(F("Clients connected:"));
      for (const auto& kv : track_last_received) {
        out.println(kv.first);
      }
      return true;
    } else if ((query == 'O') || (query == 'o')) {
      out.print(F("Outputs: [ "));
      bool first = true;
      for (const auto& pin : PIN_OUT) {
        if (first) {
          first = false;
        } else {
          out.print(F(", "));
        }
        out.print(digitalRead(pin) ? F("H") : F("L"));
      }
      out.println(F(" ]"));
      return true;
    }
  #endif
  return false;
}

void loop() {
  yield();
  if (Serial.available()) {
    char query = Serial.read();
    if ((query == 'S') || (query == 's')) {
      apply_new_row();
      disconnectWiFi(); // Disable wifi and hooks to avoid messages
      askSettings(); // noreturn
    } else if ((query == 'R') || (query == 'r')) {
      apply_new_row();
      reboot(); // noreturn
    } else if ((query == 'D') || (query == 'd')) {
      show_polling = !show_polling;
    } else {
      apply_new_row();
      if (!execQuery(query, Serial)) {
        Serial.print(F("Unknown command: "));
        Serial.println(query);
      }
    }
  }
  #ifdef MODE_SERVER
    track_older(millis(), settings.send_interval); // Could be done less frequently
  #endif
  if(WiFiConnected) {
    parsePacket();

    #ifdef MODE_CLIENT
      millis_t current_time = millis();
      if (current_time - last_send > settings.send_interval) {
        pkt_s pkt;
        memset(&pkt, 0, sizeof(pkt_s));
        pkt.magic = settings.magic;
        pkt.idx = settings.target_idx;
        pkt.value = digitalRead(PIN_IN1) ? VALUE_PIN_HIGH : VALUE_PIN_LOW;
        if (pkt.value != old_state) {
          apply_new_row();
          if(pkt.value == SWICH_CLOSED) {
            Serial.println(F("Changed swich state to closed"));
          } else {
            Serial.println(F("Changed swich state to open"));
          }
          old_state = pkt.value;
        } 
        udp.beginPacket(settings.server_ip, settings.server_port);
        lib::writeT(udp, &pkt);
        udp.endPacket();
        if (show_polling) {
          Serial.print(F("."));
          need_new_row = true;
        }
        last_send = current_time;
      }
    #endif
  }
}
