
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
    if(notPresent) {
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
      if(time - it->second > expire) { // Expire condition
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
    { D0, D1, D2, D5, D6, D7, D8 }
  #endif
  ;
  #define MIN_INTERVAL_VALUE ((millis_t)500)
#else
  #define VALUE_PIN_HIGH 0x00
  #define VALUE_PIN_LOW 0xFF
  #define SWICH_OPEN VALUE_PIN_HIGH
  #define SWICH_CLOSED VALUE_PIN_LOW
  #define MIN_INTERVAL_VALUE ((millis_t)50)
  #ifdef ESP32
    const int PIN_IN[] = { 13 };
  #else
    const int PIN_IN[] = { D1, D2, D5, D6, D7 };
  #endif
  struct send_loop_s {
    uint8_t old_state[lib::size(PIN_IN)];

    uint16_t count = 0;
    millis_t interval = 0;

    uint16_t index = 0;
    millis_t stored_time = 0;
  } send_loop;
#endif

#define LED_ON LOW
#define LED_OFF HIGH
#ifdef ESP32
  const int PIN_LED = 2;
#else
  const int PIN_LED = D4;
#endif

#define PORT_NONE 0
#define TARGET_NONE 0
struct __attribute__((packed)) server_s {
  uint8_t ip[4] = IP_ZERO;
  uint16_t port = PORT_NONE;
  uint16_t target = TARGET_NONE;
};

#include <defSettings.hpp>
#define EE_MAGIC 0xDF02
#ifdef MODE_SERVER
  #define EE_MODE 'S'
#else
  #define EE_MODE 'C'
#endif
struct __attribute__((packed)) settings_s {
  uint16_t magic = EE_MAGIC;
  char mode = EE_MODE;
  uint16_t node_id = 0;
  char ssid[64] = DEFSETTINGS_SSID;
  char pass[64] = DEFSETTINGS_PASS;
  uint8_t local_ip[4] = 
  #ifdef MODE_SERVER
    DEFSETTINGS_SERVER_IP
  #else
    IP_ZERO
  #endif
  ;
  uint8_t mask[4] = DEFSETTINGS_MASK_IP;
  uint8_t gateway[4] = DEFSETTINGS_GATEWAY_IP;
  uint16_t udp_server_port = DEFSETTINGS_SERVER_PORT;
  unsigned long send_interval = 
  #ifdef MODE_SERVER
    1000UL
  #else
    100UL
  #endif
  ;
  #ifdef MODE_CLIENT
    server_s server[lib::size(PIN_IN)];
  #endif
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

  out.print(F("Node Id: "));
  out.println(settings.node_id);

  out.print(F("Network Name (SSID): "));
  out.println(settings.ssid);

  out.print(F("Password: "));
  if(showPass) {
    out.println(settings.pass);
  } else {
    out.println(F("***"));
  }

  out.print(F("Local IP Address: "));
  out.println(IPAddress(settings.local_ip));

  out.print(F("Subnet Mask: "));
  out.println(IPAddress(settings.mask));

  out.print(F("Gateway IP Address: "));
  out.println(IPAddress(settings.gateway));

  out.print(F("Local Port: "));
  out.println(settings.udp_server_port);

  #ifdef MODE_SERVER
    out.print(F("Expire Interval: "));
  #else
    out.print(F("Send Interval: "));
  #endif
  out.print(settings.send_interval);
  out.println(F(" ms"));

  #ifdef MODE_CLIENT
    for(uint16_t i = 0; i < send_loop.count; ++i) {
      out.print(F("Server["));
      out.print(i);
      out.print(F("] IP Address: "));
      out.print(IPAddress(settings.server[i].ip));

      out.print(F(", Port: "));
      out.print(settings.server[i].port);

      out.print(F(", Target: "));
      out.println(settings.server[i].target);
    }
    out.print(F("Delay between sendings: "));
    out.print(send_loop.interval);
    out.println(F(" ms"));
  #endif
}

void settingsChanged() {
  #ifdef MODE_CLIENT
    for(send_loop.count = 0; send_loop.count < lib::size(settings.server); ++send_loop.count)
      if(!lib::isSet(IPAddress(settings.server[send_loop.count].ip)))
        break;
    send_loop.interval = (send_loop.count == 0) ? settings.send_interval : settings.send_interval / send_loop.count;
  #endif
}

WiFiUDP udp;
bool WiFiConnected = false;

[[noreturn]] void askSettings();

void reconnectWiFi() {
  if(settings.ssid[0] == '\0') {
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
  digitalWrite(PIN_LED, LED_ON);
}

void WiFiStationDisconnected(const lib::WiFiEventStationModeDisconnected& event) {
  apply_new_row();
  WiFiConnected = false;
  Serial.print(F("Disconnected from wifi for the following reason: Error -> "));
  Serial.println(event.reason);
  reconnectWiFi();
  digitalWrite(PIN_LED, LED_OFF);
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
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LED_OFF);
  Serial.begin(115200);
  delay(1000);
  #ifndef ESP32
    Serial.println();
  #endif
  Serial.println();
  EEPROM.begin(512);
  EEPROM.get(0, settings);
  if((settings.magic != EE_MAGIC) || (settings.mode != EE_MODE)) {
    // Invalid settings
    Serial.println(F("Invalid settings in EEPROM"));
    //settings = {};
    settings = settings_s();
  }
  settingsChanged();
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

  udp.begin(settings.udp_server_port); // Server for both for general info

  // Prepare pins
  #ifdef MODE_SERVER
    for(const int pin : PIN_OUT) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  #else
    for(size_t i = 0; i < lib::size(PIN_IN); i++) {
      pinMode(PIN_IN[i], INPUT_PULLUP);
      send_loop.old_state[i] = VALUE_PIN_HIGH;
    }
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
  if(pkt_len != 2)
    return false;
  if(pkt_buff[0] != '?')
    return false;

  char query = pkt_buff[1];

  lib::PrintBuff pbuff(sizeof(pkt_buff), pkt_buff);
  if(!execQuery(query, pbuff)) {
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
    if(pkt_len != sizeof(pkt_s)) {
      apply_new_row();
      Serial.print(F("Unknown packet length: "));
      Serial.println(pkt_len);
      return false;
    }
    
    pkt_s* pkt = (pkt_s*)pkt_buff;
    if(pkt->magic != PKT_MAGIC) {
      apply_new_row();
      Serial.print(F("Invalid version: 0x"));
      Serial.println(pkt->magic, HEX);
      return false;
    }

    if(pkt->idx >= lib::size(PIN_OUT)) {
      apply_new_row();
      Serial.print(F("Invalid target: "));
      Serial.println(pkt->idx);
      return false;
    }

    track_received(pkt_source, millis());

    bool prev_state = digitalRead(PIN_OUT[pkt->idx]);
    digitalWrite(PIN_OUT[pkt->idx], pkt->value ? HIGH : LOW);
    bool new_state = digitalRead(PIN_OUT[pkt->idx]);
    if(prev_state != new_state) {
      apply_new_row();
      Serial.print(F("Changed value of output "));
      Serial.print(pkt->idx);
      if(new_state)
        Serial.print(F(" to HIGH"));
      else
        Serial.print(F(" to LOW"));
      Serial.print(F("; command sent by "));
      Serial.println(pkt_source);
    } else {
      if(show_polling) {
        Serial.print(pkt->idx, HEX); //Valido per max 16 elementi 0..f
        need_new_row = true;
      }
    }

    return true;
  }
#endif

void parsePacket() {
  pkt_len = udp.parsePacket();

  if(pkt_len == 0)
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

  if(!parsed)
    parsed = parseQueryPacket();

  #ifdef MODE_SERVER
    if(!parsed)
      parsed = parsePktPacket();
  #endif

  pkt_len = 0;
}

String serialReadLine() {
  String res = "";
  while (true) {
    if(Serial.available()) {
      int c = Serial.read();
      Serial.print((char)c);
      if(c == '\n') {
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
  settings.mode = _def.mode;
  Serial.println();

  Serial.print(F("Node Id: "));
  settings.node_id = serialReadLine().toInt();

  Serial.print(F("Network Name (SSID): "));
  serialReadLine().toCharArray(settings.ssid, sizeof(settings.ssid) - 1);

  Serial.print(F("Password: "));
  serialReadLine().toCharArray(settings.pass, sizeof(settings.pass) - 1);

  Serial.print(F("Local IP Address: "));
  serialReadIP(settings.local_ip);

  if(lib::isSet(IPAddress(settings.local_ip))) {
    Serial.print(F("Subnet Mask: "));
    serialReadIP(settings.mask);

    Serial.print(F("Gateway IP Address: "));
    serialReadIP(settings.gateway);
  }

  Serial.print(F("Local Port: "));
  settings.udp_server_port = serialReadLine().toInt();

  #ifdef MODE_SERVER
    Serial.print(F("Expire Interval: "));
  #else
    Serial.print(F("Send Interval: "));
  #endif
  settings.send_interval = serialReadLine().toInt();
  settings.send_interval = max(settings.send_interval, MIN_INTERVAL_VALUE);

  #ifdef MODE_CLIENT
    for(uint16_t i = 0; i < lib::size(settings.server); ++i) {
      Serial.print(F("Server["));
      Serial.print(i);
      Serial.print(F("] IP Address: "));
      serialReadIP(settings.server[i].ip);
      if(!lib::isSet(IPAddress(settings.server[i].ip))) {
        break;
      }
      Serial.print(F("Server["));
      Serial.print(i);
      Serial.print(F("] Port: "));
      settings.server[i].port = serialReadLine().toInt();

      Serial.print(F("Server["));
      Serial.print(i);
      Serial.print(F("] Target: "));
      settings.server[i].target = serialReadLine().toInt();
    }
  #endif

  settingsChanged();

  Serial.println();
  printSettings(Serial, true);
  Serial.print(F("Save? [N/y]: "));
  s = serialReadLine();
  s.toUpperCase();
  if(s == "Y") {
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
  if(query == 'I') {
    printSettings(out, false);
    return true;
  } else if(query == '+') {
    if(WiFi.isConnected()) {
      out.print(F("Network Name (SSID): "));
      out.println(WiFi.SSID());
      out.print(F("WiFi Channel = "));
      out.print(WiFi.channel());
      out.print(F(", Signal = "));
      out.print(WiFi.RSSI());
      out.println(F(" dbm"));
      out.print(F("Local IP Address: "));
      out.println(WiFi.localIP());
      out.print(F("Subnet Mask: "));
      out.println(WiFi.subnetMask());
      out.print(F("Gateway IP Address: "));
      out.println(WiFi.gatewayIP());
    } else {
      out.println(F("WiFi <NOT CONNECTED>"));
    }
    return true;
  } else if(query == 'W') {
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
    else if(query == 'C') {
      out.println(F("Clients connected:"));
      for (const auto& kv : track_last_received) {
        out.println(kv.first);
      }
      return true;
    } else if(query == 'O') {
      out.print(F("Outputs: [ "));
      bool first = true;
      for (const auto& pin : PIN_OUT) {
        if(first) {
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

#ifdef MODE_CLIENT

  void packetSendLoop() {
    millis_t current_time = millis();
    if(current_time - send_loop.stored_time < send_loop.interval)
      return; // Not time yet

    if(send_loop.index >= send_loop.count) {
      // Either no server or broken internal send_loop?
      send_loop.index = 0;
      send_loop.stored_time = current_time; // Update time anyway so you don't get this every frame
      return;
    }

    // Prepare packet
    pkt_s pkt;
    memset(&pkt, 0, sizeof(pkt_s));
    pkt.magic = settings.magic;
    pkt.idx = settings.server[send_loop.index].target;
    pkt.value = digitalRead(PIN_IN[send_loop.index]) ? VALUE_PIN_HIGH : VALUE_PIN_LOW;

    // Print if state changed
    if(pkt.value != send_loop.old_state[send_loop.index]) {
      apply_new_row();
      Serial.print(F("Changed swich["));
      Serial.print(send_loop.index);
      Serial.print(F("] state to "));
      Serial.println((pkt.value == SWICH_CLOSED) ? F("closed") : F("open"));
      send_loop.old_state[send_loop.index] = pkt.value;
    }

    // Send packet
    udp.beginPacket(settings.server[send_loop.index].ip, settings.server[send_loop.index].port);
    lib::writeT(udp, &pkt);
    udp.endPacket();

    ++send_loop.index;
    if(send_loop.index >= send_loop.count)
      send_loop.index = 0;

    if(show_polling) {
      Serial.print(F("."));
      need_new_row = true;
    }
    send_loop.stored_time = current_time;
  }

#endif

void loop() {
  yield();
  if(Serial.available()) {
    char query = toupper(Serial.read());
    if(query == 'S') {
      apply_new_row();
      disconnectWiFi(); // Disable wifi and hooks to avoid messages
      askSettings(); // noreturn
    } else if(query == 'R') {
      apply_new_row();
      reboot(); // noreturn
    } else if(query == 'D') {
      show_polling = !show_polling;
    } else {
      apply_new_row();
      if(!execQuery(query, Serial)) {
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
      packetSendLoop();
    #endif
  }
}
