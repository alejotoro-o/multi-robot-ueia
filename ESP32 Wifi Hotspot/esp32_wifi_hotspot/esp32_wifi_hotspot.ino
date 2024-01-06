#include <WiFi.h>
#include "esp_wifi.h"

const char* ssid           = "ESP32_WIFI_HOTSPOT";                // SSID Name
const char* password       = "abcd1234";   // SSID Password - Set to NULL to have an open AP
const int   channel        = 10;                        // WiFi Channel number between 1 and 13
const bool  hide_SSID      = false;                     // To disable SSID broadcast -> SSID will not appear in a basic WiFi scan
const int   max_connection = 4;                         // Maximum simultaneous connected clients on the AP

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, channel, hide_SSID, max_connection);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());
}

void loop()
{

}
