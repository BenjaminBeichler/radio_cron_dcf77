#ifndef WIFI_H
#define WIFI_H

// Arrays of SSIDs and passwords
const char* WIFI_SSIDS[] = {
  "SSID-1",
  "SSID-2",
  "SSID-3"

};

const char* WIFI_PASSWORDS[] = {
  "PASSWORD-1",  // password for "SSID-1"
  "PASSWORD-2",   // password for "SSID-2"
  "PASSWORD-3"   // password for "SSID-3"

};

// Number of networks
const int WIFI_NETWORK_COUNT = sizeof(WIFI_SSIDS) / sizeof(WIFI_SSIDS[0]);

// NTP settings and time zone information
const char* ntpServer = "by.pool.ntp.org";
const char* TZ_INFO   = "MSK-3MSD,M3.5.0/2,M10.5.0/3";

#endif // WIFI_H
