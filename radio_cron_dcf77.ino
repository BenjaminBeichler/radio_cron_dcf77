/*
  DCF77 Transmitter with Scheduled Sync Windows and Initial 20-Minute Active Period
  ------------------------------------------------------------------------------------
  The ESP32 transmits the DCF77 signal only during specified time windows:
    00:00, 01:30–01:40, 02:00–02:10, 03:00–03:10, 04:00–04:10,
    05:00–05:10, 06:00–06:10, 09:30–09:40, 17:30–17:40.
  At all other times, the device goes into deep sleep.
  
  When powered on, the device remains active for 20 minutes (initial period).
  Then, if the current time is not within one of the synchronization windows,
  the ESP32 goes into deep sleep until the beginning of the next window.

  If the CONTINUOUSMODE macro is defined, the device runs continuously and does not go to deep sleep.
*/

#include <WiFi.h>
#include <Ticker.h>
#include <Time.h>   // Depending on your environment, you may still need this
#include <time.h>
#include "wifi.h"   // Includes multiple networks: WIFI_SSIDS[], WIFI_PASSWORDS[], etc.

// ----------------------
// Pin and constant definitions
// ----------------------
#define LEDBUILTIN 2      // Pin for the LED (indicates active transmission)
#define ANTENNAPIN 18     // Pin for the antenna connection (through a 1kΩ resistor, then GND)

// If you want the device to run continuously, uncomment the following line:
// #define CONTINUOUSMODE

// ----------------------
// Global variables
// ----------------------
struct tm timeinfo;          // Structure for storing local time
const int pwmChannel = 0;    // PWM channel for ledc

Ticker tickerDecisec;        // Ticker object to call DcfOut function every 100 ms

// Array of pulses to form the DCF77 signal (60 seconds)
int impulseArray[60];
int impulseCount = 0;
int actualHours, actualMinutes, actualSecond, actualDay, actualMonth, actualYear, DayOfW;

// The total time we allow for WiFi connection or initial active period
long dontGoToSleep = 0;                // ESP32 startup time (in milliseconds)
const long onTimeAfterReset = 1200000;  // 20 minutes in milliseconds
int timeRunningContinuous = 0;          // Counter for continuous transmission mode

// ----------------------
// Functions for WiFi and NTP
// ----------------------

// This function tries one pass over all networks; returns true if connected
// and false if it failed to connect to every network in the list.
bool WiFi_on() {
  Serial.println("=== WiFi ON ===");
  WiFi.mode(WIFI_STA);

  bool connected = false;
  unsigned long startAttemptTime;

  for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
    Serial.print("Connecting to WiFi network: ");
    Serial.println(WIFI_SSIDS[i]);

    WiFi.begin(WIFI_SSIDS[i], WIFI_PASSWORDS[i]);
    startAttemptTime = millis();

    // Give 15 seconds to connect to this network
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime) < 15000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;  // Stop searching if we have connected successfully
    } else {
      Serial.println("Failed to connect. Trying the next network...");
    }
  }

  if (connected) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to any network in this pass.");
  }
  return connected;
}

void getNTP() {
  Serial.println("=== Getting NTP time ===");
  // Set system time via NTP (UTC)
  configTime(0, 0, ntpServer);
  // Apply the time zone settings from wifi.h
  setenv("TZ", TZ_INFO, 1);
  tzset();
  
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error: Failed to obtain time from NTP");
  } else {
    Serial.printf("NTP time updated. Local time: %02d:%02d:%02d\n",
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
}

void WiFi_off() {
  Serial.println("Turning WiFi off...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi is off.");
}

void show_time() {
  Serial.printf("Current Local Time: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

// ----------------------
// DCF77 signal generation
// ----------------------

// Convert a decimal number to BCD
int Bin2Bcd(int dato) {
  int msb, lsb;
  if (dato < 10)
    return dato;
  msb = (dato / 10) << 4;
  lsb = dato % 10;
  return msb + lsb;
}

// The CodeTime() function forms the impulseArray for the DCF77 signal
void CodeTime() {
  // Determine the day of the week (0 -> 7 for DCF77)
  DayOfW = timeinfo.tm_wday;
  if (DayOfW == 0) DayOfW = 7;
  
  actualDay    = timeinfo.tm_mday;
  actualMonth  = timeinfo.tm_mon + 1;
  actualYear   = timeinfo.tm_year - 100;  // use a 2-digit year
  actualHours  = timeinfo.tm_hour;
  // DCF77 transmits time for the next minute
  actualMinutes = timeinfo.tm_min + 1;
  if (actualMinutes >= 60) {
    actualMinutes = 0;
    actualHours++;
  }
  actualSecond = timeinfo.tm_sec;
  if (actualSecond == 60) actualSecond = 0;

  int n, Tmp, TmpIn;
  int ParityCount = 0;

  // First 20 seconds – logical "0" (100 ms pulse)
  for (n = 0; n < 20; n++) {
    impulseArray[n] = 1;
  }

  // Set bits for DST: if tm_isdst == 0, DST is off
  if (timeinfo.tm_isdst == 0) {
    impulseArray[18] = 2;  // 200 ms pulse – DST OFF
  } else {
    impulseArray[17] = 2;  // 200 ms pulse – DST ON
  }

  // Bit 20 – active time indicator
  impulseArray[20] = 2;

  // Form bits for minutes (bits 21..27) and parity bit (28)
  ParityCount = 0;
  TmpIn = Bin2Bcd(actualMinutes);
  for (n = 21; n < 28; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  impulseArray[28] = ((ParityCount & 1) == 0) ? 1 : 2;

  // Form bits for hours (bits 29..34) and parity bit (35)
  ParityCount = 0;
  TmpIn = Bin2Bcd(actualHours);
  for (n = 29; n < 35; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  impulseArray[35] = ((ParityCount & 1) == 0) ? 1 : 2;

  // Form bits for the date: day, day of the week, month, year, and the parity bit (58)
  ParityCount = 0;
  TmpIn = Bin2Bcd(actualDay);
  for (n = 36; n < 42; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(DayOfW);
  for (n = 42; n < 45; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(actualMonth);
  for (n = 45; n < 50; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(actualYear);
  for (n = 50; n < 58; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  impulseArray[58] = ((ParityCount & 1) == 0) ? 1 : 2;

  // The last second – no pulse
  impulseArray[59] = 0;
}

// The DcfOut() function is called every 100 ms and generates the DCF77 signal
void DcfOut() {
  switch (impulseCount++) {
    case 0:
      if (impulseArray[actualSecond] != 0) {
        digitalWrite(LEDBUILTIN, LOW);
        ledcWrite(pwmChannel, 0);
      }
      break;
    case 1:
      if (impulseArray[actualSecond] == 1) {
        digitalWrite(LEDBUILTIN, HIGH);
        ledcWrite(pwmChannel, 127);
      }
      break;
    case 2:
      digitalWrite(LEDBUILTIN, HIGH);
      ledcWrite(pwmChannel, 127);
      break;
    case 9:
      impulseCount = 0;
      // Print bit information for the current second to the console
      if (actualSecond == 1 || actualSecond == 15 ||
          actualSecond == 21 || actualSecond == 29)
        Serial.print("-");
      if (actualSecond == 36 || actualSecond == 42 ||
          actualSecond == 45 || actualSecond == 50)
        Serial.print("-");
      if (actualSecond == 28 || actualSecond == 35 ||
          actualSecond == 58)
        Serial.print("P");
      if (impulseArray[actualSecond] == 1)
        Serial.print("0");
      if (impulseArray[actualSecond] == 2)
        Serial.print("1");
      if (actualSecond == 59) {
        Serial.println();
        show_time();
      }
      break;
  }
  // Update time and recalculate the pulse array
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error obtaining time...");
    delay(3000);
    ESP.restart();
  }
  CodeTime();
}

// ----------------------
// Sync windows and deep sleep logic
// ----------------------

// Structure of a sync window (start time and duration)
struct SyncWindow {
  int hour;   // Start hour
  int minute; // Start minute
};

// Define the synchronization windows
// (each window lasted 20 minutes in the original code; now modified to 10 minutes)
// Added new window at 00:00.
const SyncWindow syncWindows[] = {
  {0, 0},
  {1, 30},
  {2, 0},
  {3, 0},
  {4, 0},
  {5, 0},
  {6, 0},
  {9, 30},
  {17, 30}
};
const int numSyncWindows = sizeof(syncWindows) / sizeof(syncWindows[0]);

// Checks if the current time is within one of the sync windows
bool isSyncWindowActive() {
  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  for (int i = 0; i < numSyncWindows; i++) {
    int start = syncWindows[i].hour * 60 + syncWindows[i].minute;
    int end = start + 10; // each window now lasts 10 minutes
    if (nowMinutes >= start && nowMinutes < end) {
      Serial.printf("Sync window active: %02d:%02d to %02d:%02d\n",
                    syncWindows[i].hour, syncWindows[i].minute,
                    syncWindows[i].hour, syncWindows[i].minute + 10);
      return true;
    }
  }
  return false;
}

// Calculates the time (in seconds) until the start of the next sync window
unsigned long secondsToNextSyncWindow() {
  int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int minDiff = 24 * 60; // maximum value for a day
  for (int i = 0; i < numSyncWindows; i++) {
    int start = syncWindows[i].hour * 60 + syncWindows[i].minute;
    int diff = start - nowMinutes;
    if (diff < 0) diff += 24 * 60; // if the window has already passed, add a day
    if (diff < minDiff) {
      minDiff = diff;
    }
  }
  Serial.printf("Next sync window in %d minutes (~%lu seconds)\n", minDiff, minDiff * 60UL);
  return minDiff * 60UL; // convert minutes to seconds
}

// Goes into deep sleep if outside the sync window (unless CONTINUOUSMODE is defined)
void checkSleep() {
#ifdef CONTINUOUSMODE
  Serial.println("Continuous mode enabled. Skipping sleep check.");
  return;
#else
  // If more than 20 minutes have passed since power on, check the sync window
  if (millis() - dontGoToSleep > onTimeAfterReset) {
    if (!isSyncWindowActive()) {
      unsigned long sleepSeconds = secondsToNextSyncWindow();
      Serial.printf("Outside sync window. Going to deep sleep for %lu seconds...\n", sleepSeconds);
      ESP.deepSleep(sleepSeconds * 1000000ULL);
    } else {
      Serial.println("Within sync window. Staying awake.");
    }
  } else {
    Serial.println("Initial 20-minute active period. Staying awake.");
  }
#endif
}

// ----------------------
// setup() and loop()
// ----------------------
void setup() {
  // Disable wake-up from other sources
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== DCF77 Transmitter with Scheduled Sync Windows ===");

  // Record the time the device was started (not from deep sleep)
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    dontGoToSleep = millis();
    Serial.printf("Device started at millis: %lu\n", dontGoToSleep);
  }

  // Keep trying to connect to WiFi for up to 20 minutes
  bool connected = false;
  while ((millis() - dontGoToSleep) < onTimeAfterReset) {
    if (WiFi_on()) {
      // If we got connected during this pass, break
      connected = true;
      break;
    } else {
      // Failed to connect in this pass — wait a bit before retrying
      Serial.println("Will try again in 5 seconds...");
      delay(5000);
    }
  }

  // If, after 20 minutes, we are still not connected, go to deep sleep
  if (!connected) {
    Serial.println("No WiFi connection after 20 minutes. Going to deep sleep...");
    // You can choose how long to sleep (e.g., 1 hour) or go back to scheduling logic
    // For now, let's just deep sleep for 1 hour as an example:
    ESP.deepSleep(3600ULL * 1000000ULL);
  }

  // Otherwise, if we are connected, proceed with NTP sync
  getNTP();
  WiFi_off();
  show_time();

#ifndef CONTINUOUSMODE
  // If more than 20 minutes have passed and we're outside the sync window, go to deep sleep
  checkSleep();
#else
  Serial.println("Continuous mode active. Device will not enter deep sleep.");
#endif

  // Configure PWM for the DCF77 signal
  ledcSetup(pwmChannel, 77500, 8); // 77.5 kHz, 8-bit resolution
  ledcAttachPin(ANTENNAPIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  pinMode(LEDBUILTIN, OUTPUT);
  digitalWrite(LEDBUILTIN, LOW);

  // Build the initial DCF77 pulse array
  CodeTime();

  // Synchronize with the start of a second for accurate transmission
  Serial.print("Syncing with start of a second... ");
  int startSecond = timeinfo.tm_sec;
  long count = 0;
  while (true) {
    count++;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Error obtaining time...");
      delay(3000);
      ESP.restart();
    }
    if (timeinfo.tm_sec != startSecond) break;
  }
  Serial.print("Synced after ");
  Serial.print(count);
  Serial.println(" checks.");

  // Start the Ticker which calls DcfOut() every 100 ms
  tickerDecisec.attach_ms(100, DcfOut);
}

void loop() {
#ifndef CONTINUOUSMODE
  // Every 30 seconds, check if the sync window has ended
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 30000) {
    lastCheck = millis();
    Serial.println("Periodic check of sync window...");
    // If the initial 20-minute period has passed
    if (millis() - dontGoToSleep > onTimeAfterReset) {
      if (!isSyncWindowActive()) {
        Serial.println("Sync window ended. Preparing to enter deep sleep.");
        if (getLocalTime(&timeinfo)) {
          checkSleep();
        }
      } else {
        Serial.println("Still within sync window. Continuing operation.");
      }
    } else {
      Serial.println("Within the initial 20-minute active period.");
    }
  }
#endif
  // All other work is performed via the Ticker (DcfOut function)
}
