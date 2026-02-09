#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>
#include <helpers/sensors/MicroNMEALocationProvider.h>

T114Board board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
T114SensorManager sensors = T114SensorManager(nmea);

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

bool radio_init() {
  rtc_clock.begin(Wire);

  return radio.std_init(&SPI);
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(uint8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

void T114SensorManager::start_gps() {
  if (!gps_active) {
    gps_active = true;
    _location->begin();
  }
}

void T114SensorManager::stop_gps() {
  if (gps_active) {
    gps_active = false;
    _location->stop();
  }
}

bool T114SensorManager::begin() {
  Serial1.begin(9600);
  Serial2.begin(115200);  // C6 UART on pins 9/10

  // Try to detect if GPS is physically connected to determine if we should expose the setting
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);  // Power on GPS

  // Give GPS a moment to power up and send data
  delay(1500);

  // We'll consider GPS detected if we see any data on Serial1
  gps_detected = (Serial1.available() > 0);

  if (gps_detected) {
    MESH_DEBUG_PRINTLN("GPS detected");
  } else {
    MESH_DEBUG_PRINTLN("No GPS detected");
  }
  digitalWrite(GPS_EN, LOW);  // Power off GPS until the setting is changed

  return true;
}

bool T114SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  if (c6_data_valid && (requester_permissions & TELEM_PERM_ENVIRONMENT)) {
    telemetry.addTemperature(TELEM_CHANNEL_SELF, c6_temperature);
    telemetry.addRelativeHumidity(TELEM_CHANNEL_SELF, c6_humidity);
    telemetry.addBarometricPressure(TELEM_CHANNEL_SELF, c6_pressure);
  }
  return true;
}

void T114SensorManager::handleC6Line(const char* line) {
  Serial.print("[C6 RX] ");
  Serial.println(line);

  if (strncmp(line, "DATA:", 5) == 0) {
    const char* p = line + 5;
    while (*p) {
      if (p[0] == 'T' && p[1] == ':') c6_temperature = atof(p + 2);
      else if (p[0] == 'H' && p[1] == ':') c6_humidity = atof(p + 2);
      else if (p[0] == 'P' && p[1] == ':') c6_pressure = atof(p + 2);
      while (*p && *p != ',') p++;
      if (*p == ',') p++;
    }
    c6_data_valid = true;
    Serial.printf("[C6] DATA: T=%.1f H=%.1f P=%.1f\n", c6_temperature, c6_humidity, c6_pressure);
  }
  else if (strncmp(line, "ALERT:", 6) == 0) {
    strncpy(c6_alert, line + 6, sizeof(c6_alert) - 1);
    c6_alert[sizeof(c6_alert) - 1] = '\0';
    c6_alert_pending = true;
    Serial.printf("[C6] ALERT queued: %s\n", c6_alert);
  }
  else if (strncmp(line, "CMD:", 4) == 0) {
    const char* cmd = line + 4;
    Serial.printf("[C6] CMD: %s\n", cmd);

    if (strcmp(cmd, "PING") == 0) {
      Serial2.println("RSP:PONG");
    }
    else if (strcmp(cmd, "STATUS") == 0) {
      Serial2.printf("RSP:STATUS:T=%.1f,H=%.1f,P=%.1f,V=%.2f\n",
        c6_temperature, c6_humidity, c6_pressure,
        (float)board.getBattMilliVolts() / 1000.0f);
    }
    else if (strcmp(cmd, "BATT") == 0) {
      Serial2.printf("RSP:BATT:%.2f\n", (float)board.getBattMilliVolts() / 1000.0f);
    }
    else {
      Serial2.printf("RSP:ERR:unknown cmd '%s'\n", cmd);
    }
  }
  else {
    Serial.println("[C6] Unknown line format, ignored");
  }
}

void T114SensorManager::loop() {
  // Read C6 UART
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      c6_buf[c6_pos] = '\0';
      handleC6Line(c6_buf);
      c6_pos = 0;
    } else if (c != '\r' && c6_pos < sizeof(c6_buf) - 1) {
      c6_buf[c6_pos++] = c;
    }
  }

  static long next_gps_update = 0;

  _location->loop();

  if (millis() > next_gps_update) {
    if (_location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
    }
    next_gps_update = millis() + 1000;
  }
}

int T114SensorManager::getNumSettings() const {
  return gps_detected ? 1 : 0;  // only show GPS setting if GPS is detected
}

const char* T114SensorManager::getSettingName(int i) const {
  return (gps_detected && i == 0) ? "gps" : NULL;
}

const char* T114SensorManager::getSettingValue(int i) const {
  if (gps_detected && i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}

bool T114SensorManager::setSettingValue(const char* name, const char* value) {
  if (gps_detected && strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      stop_gps();
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}
