#include "ODriveCluster.h"

// Instantiate our helper class
ODriveCluster motors;

void setup() {
  Serial.begin(115200);
  
  // 1. Start CAN Hardware
  if (!motors.begin(PIN_CAN_STANDBY, PIN_CAN_BOOSTEN)) {
    Serial.println("CAN Init Failed!");
    while(1);
  }
  Serial.println("CAN Started.");

  // 2. Discover Motors (Wait up to 15 seconds)
  motors.waitForDiscovery(15000);

  // 3. Configure and Activate Found Motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motors.isConnected(i)) {
      Serial.print("Activating Node "); Serial.println(i);
      
      motors.enableClosedLoop(i);
      motors.configurePID(i, 40.0f, 0.3f, 0.16f);
      
      delay(20); // Stagger startup
    }
  }
}

void loop() {
  // 1. Process Incoming Messages (ALWAYS DO THIS)
  motors.processCAN();

  // 2. Send Commands (100Hz)
  static uint32_t last_command_time = 0;
  if (millis() - last_command_time >= 10) {
    last_command_time = millis();
    
    float t = millis() / 1000.0f;

    for (int i = 0; i < NUM_MOTORS; i++) {
      if (motors.isConnected(i)) {
        float phase_offset = i * (PI / 2.0f);
        float target_pos = sin(t + phase_offset);
        
        motors.setPosition(i, target_pos);
      }
    }
  }

  // 3. Print Dashboard (10Hz)
  static uint32_t last_print_time = 0;
  if (millis() - last_print_time >= 100) {
    last_print_time = millis();
    motors.printStatus();
  }
}