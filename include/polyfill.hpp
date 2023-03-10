#ifndef _POLYFILL_HPP
#define _POLYFILL_HPP

#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

namespace lib {
    #ifdef ESP32
        template<typename T, int N>
        inline int size(T(&)[N]) { return N; }
    #else
        using std::size;
    #endif

    inline const uint32_t v4(const IPAddress& ip) {
        #ifdef ESP32
            return (uint32_t)ip;
        #else
            return ip.v4();
        #endif
    }

    inline bool isSet(const IPAddress& ip) {
        #ifdef ESP32
            return ip != INADDR_NONE;
        #else
            return ip.isSet();
        #endif
    }

    #ifdef ESP32
        struct WiFiEventStationModeConnected {};
        struct WiFiEventStationModeGotIP {};
        struct WiFiEventStationModeDisconnected {};
        class WiFiEventHandlerOpaque {
            private:
                WiFiClass& _WiFi;
                wifi_event_id_t _id;
            public:
                WiFiEventHandlerOpaque(WiFiClass& _WiFi, wifi_event_id_t _id) : _WiFi(_WiFi), _id(_id) {}
                virtual ~WiFiEventHandlerOpaque() {
                    _WiFi.removeEvent(_id);
                }
        };
        using WiFiEventHandler = std::shared_ptr<WiFiEventHandlerOpaque>;
        inline WiFiEventHandler onStationModeConnected(WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeConnected&)> f) {
            auto id = _WiFi.onEvent([f] (arduino_event_id_t event, arduino_event_info_t info) { f({}); }, ARDUINO_EVENT_WIFI_STA_CONNECTED);
            return std::make_shared<WiFiEventHandlerOpaque>(_WiFi, id);
        }
        inline WiFiEventHandler onStationModeGotIP(WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeGotIP&)> f) {
            auto id = _WiFi.onEvent([f] (arduino_event_id_t event, arduino_event_info_t info) { f({}); }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
            return std::make_shared<WiFiEventHandlerOpaque>(_WiFi, id);
        }
        inline WiFiEventHandler onStationModeDisconnected(WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeDisconnected&)> f) {
            auto id = _WiFi.onEvent([f] (arduino_event_id_t event, arduino_event_info_t info) { f({}); }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
            return std::make_shared<WiFiEventHandlerOpaque>(_WiFi, id);
        }
    #else
        using ::WiFiEventStationModeConnected;
        using ::WiFiEventStationModeGotIP;
        using ::WiFiEventStationModeDisconnected;
        using ::WiFiEventHandler;
        inline WiFiEventHandler onStationModeConnected(ESP8266WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeConnected&)> f) {
            return _WiFi.onStationModeConnected(f);
        }
        inline WiFiEventHandler onStationModeGotIP(ESP8266WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeGotIP&)> f) {
            return _WiFi.onStationModeGotIP(f);
        }
        inline WiFiEventHandler onStationModeDisconnected(ESP8266WiFiClass& _WiFi, std::function<void(const WiFiEventStationModeDisconnected&)> f) {
            return _WiFi.onStationModeDisconnected(f);
        }
    #endif
}

#endif
