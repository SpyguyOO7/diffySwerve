#ifndef IBUS_INPUT_H
#define IBUS_INPUT_H

#include <Arduino.h>

// --- Configuration ---
#define IBUS_SERIAL       Serial1      // Feather M4 RX/TX
#define IBUS_BAUD         115200
#define IBUS_MAX_CHANNELS 14
#define IBUS_TIMEOUT_MS   100          // SAFETY CRITICAL: Stop within ~100ms of signal loss

// --- Safety Settings ---
// Tight tolerance allows for tiny TX jitter (e.g. 1999 vs 2000)
// but prevents random values from triggering states.
#define SWITCH_TOLERANCE  10           

// --- Limits ---
#define MAX_VEL_X         0.5f         // m/s
#define MAX_VEL_Y         0.5f         // m/s
#define MAX_OMEGA         0.1f         // rad/s (approx 90 deg/s)
#define DEADZONE          30           // PWM units (around 1500)

enum SystemState {
  STATE_DISABLED = 0, // Motors OFF (Safe/Brake)
  STATE_STANDBY  = 1, // Motors ON (Closed Loop), Vel = 0
  STATE_ACTIVE   = 2  // Motors ON, Kinematics Active
};

class IBusInput {
  public:
    void begin() {
      IBUS_SERIAL.begin(IBUS_BAUD);
    }

    void update() {
      while (IBUS_SERIAL.available()) {
        uint8_t val = IBUS_SERIAL.read();
        
        // i-BUS Protocol Parser (0x20 0x40 Header)
        if (state == 0 && val == 0x20) { state = 1; buffer[0] = val; }
        else if (state == 1 && val == 0x40) { state = 2; buffer[1] = val; idx = 2; }
        else if (state == 2) {
          buffer[idx++] = val;
          if (idx == 32) { // Packet Complete
            parseChannelData();
            state = 0;
            last_packet_time = millis();
          }
        } else {
          state = 0; // Reset on garbage
        }
      }
    }

    // --- Strict Safety Logic ---
    SystemState getSystemState() {
      // 1. Timeout Check (Critical)
      if (millis() - last_packet_time > IBUS_TIMEOUT_MS) return STATE_DISABLED;

      // 2. Arm Switch (Ch5) - MUST BE EXACTLY 1000 (ON)
      // "Switch off at 1000, on at 2000"
      if (!isCloseTo(channels[4], 2000)) return STATE_DISABLED;

      // 3. Mode Switch (Ch6) - Strict Checks
      // "1500 = Standby, 2000 = Active"
      // Anything else (including 1000) = Disabled/Safe
      if (isCloseTo(channels[5], 2000)) return STATE_ACTIVE;
      if (isCloseTo(channels[5], 1500)) return STATE_STANDBY;

      // Default for any other value (e.g. 1000, or corrupted)
      return STATE_DISABLED; 
    }

    // --- Control Inputs ---
    float getVx() {
      if (getSystemState() != STATE_ACTIVE) return 0.0f;
      return mapInputToMetric(channels[0], MAX_VEL_X);
    }

    float getVy() {
      if (getSystemState() != STATE_ACTIVE) return 0.0f;
      return mapInputToMetric(channels[1], MAX_VEL_Y);
    }

    float getOmega() {
      if (getSystemState() != STATE_ACTIVE) return 0.0f;
      // Note: User specified Ch3 for Angular. channels[2] is Ch3.
      return mapInputToMetric(channels[2], MAX_OMEGA); 
    }

    // --- Debug ---
    // Returns raw PWM (1000-2000) for debugging
    uint16_t getRawChannel(int ch) {
        if (ch >= 0 && ch < IBUS_MAX_CHANNELS) return channels[ch];
        return 0;
    }

  private:
    uint16_t channels[IBUS_MAX_CHANNELS];
    uint8_t buffer[32];
    int state = 0;
    int idx = 0;
    uint32_t last_packet_time = 0;

    void parseChannelData() {
      for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
        // i-BUS is little endian
        channels[i] = buffer[2 + 2*i] | (buffer[3 + 2*i] << 8);
      }
    }

    // Helper for "Precise Value" check
    bool isCloseTo(uint16_t val, uint16_t target) {
      return abs((int)val - (int)target) <= SWITCH_TOLERANCE;
    }

    float mapInputToMetric(uint16_t raw, float max_val) {
      int16_t val = (int16_t)raw - 1500;
      
      // Deadzone
      if (abs(val) < DEADZONE) return 0.0f;

      // Normalize (+/- 500)
      float norm = (float)val / 500.0f;
      
      // Clamp
      if (norm > 1.0f) norm = 1.0f;
      if (norm < -1.0f) norm = -1.0f;

      return norm * max_val;
    }
};

#endif