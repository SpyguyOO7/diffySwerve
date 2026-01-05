#include "ODriveCluster.h"
#include "DiffyKinematics.h"
#include "IBusInput.h"

ODriveCluster motors;
DiffyKinematics kinematics;
IBusInput rc;

const int POD_MOTORS[3][2] = {{0, 1}, {2, 3}, {4, 5}};

float m1_offsets[3] = {0, 0, 0};
float m2_offsets[3] = {0, 0, 0};
SwerveState latest_targets[3];
SystemState last_sys_state = STATE_DISABLED;

const float SLEW_RATE = 2.0f; 
float slew_vx = 0.0f;
float slew_vy = 0.0f;
float slew_omega = 0.0f;

void setup() {
  Serial.begin(115200);
  rc.begin(); 

  if (!motors.begin(PIN_CAN_STANDBY, PIN_CAN_BOOSTEN)) {
    Serial.println("CAN Init Failed!");
    while(1);
  }
  Serial.println("CAN Started.");
  motors.waitForDiscovery(10000); 

  Serial.println("Zeroing... (Align Wheels FORWARD)");
  for (int i = 0; i < 3; i++) {
     int m1 = POD_MOTORS[i][0];
     int m2 = POD_MOTORS[i][1];
     if (motors.isConnected(m1) && motors.isConnected(m2)) {
       m1_offsets[i] = motors.getPosition(m1);
       m2_offsets[i] = motors.getPosition(m2);

       motors.setControlMode(m1, 2, 1);
       motors.setControlMode(m2, 2, 1);
       // Start in IDLE (Blue LED)
       motors.setAxisState(m1, 1); 
       motors.setAxisState(m2, 1);
     }
  }
  Serial.println("System Ready. Waiting for Receiver...");
}

void loop() {
  motors.processCAN();
  rc.update();

  static uint32_t last_ctrl = 0;
  uint32_t now = millis();
  float dt = (now - last_ctrl) / 1000.0f;

  if (dt >= 0.02f) {
    last_ctrl = now;

    // --- 1. GLOBAL SAFETY CHECK ---
    bool system_healthy = true;
    for(int i=0; i<6; i++) {
        if (!motors.isConnected(i)) system_healthy = false;
    }

    SystemState current_state = rc.getSystemState();

    if (!system_healthy) {
        current_state = STATE_DISABLED;
    }

    // --- 2. STATE TRANSITIONS ---
    if (current_state != last_sys_state) {
        Serial.print("State Change: "); Serial.println(current_state);
        
        // ENTERING STANDBY or ACTIVE -> GO GREEN (Closed Loop)
        if (current_state >= STATE_STANDBY && last_sys_state == STATE_DISABLED) {
             for(int i=0; i<6; i++) {
                if(motors.isConnected(i)) motors.enableClosedLoop(i);
             }
        }
        // ENTERING DISABLED -> GO BLUE (Idle / Coast)
        else if (current_state == STATE_DISABLED) {
             for(int i=0; i<6; i++) {
                if(motors.isConnected(i)) motors.setAxisState(i, 1); // 1 = IDLE
             }
             // Reset Slew
             slew_vx = 0; slew_vy = 0; slew_omega = 0;
        }
    }
    last_sys_state = current_state;

    // --- 3. INPUTS & SLEW ---
    float target_vx = 0.0f; 
    float target_vy = 0.0f;
    float target_omega = 0.0f; 

    if (current_state == STATE_ACTIVE) {
        target_vx = rc.getVx();
        target_vy = rc.getVy();
        target_omega = rc.getOmega();
    } 

    float max_step = SLEW_RATE * dt;
    if (target_vx > slew_vx) slew_vx = min(target_vx, slew_vx + max_step);
    else slew_vx = max(target_vx, slew_vx - max_step);
    if (target_vy > slew_vy) slew_vy = min(target_vy, slew_vy + max_step);
    else slew_vy = max(target_vy, slew_vy - max_step);
    if (target_omega > slew_omega) slew_omega = min(target_omega, slew_omega + max_step);
    else slew_omega = max(target_omega, slew_omega - max_step);

    // --- 4. MOTOR EXECUTION ---
    for (int i = 0; i < 3; i++) {
        int m1 = POD_MOTORS[i][0];
        int m2 = POD_MOTORS[i][1];
        if (!motors.isConnected(m1) || !motors.isConnected(m2)) continue;

        if (current_state == STATE_DISABLED) {
             // Do nothing. Motors are in IDLE mode.
        } 
        else if (current_state == STATE_STANDBY) {
             // ACTIVE BRAKE (Green LED, Holding Position)
             motors.setVelocity(m1, 0.0f);
             motors.setVelocity(m2, 0.0f);
        }
        else {
             // ACTIVE RUN
             float p1 = motors.getPosition(m1) - m1_offsets[i];
             float p2 = motors.getPosition(m2) - m2_offsets[i];
             float current_angle = kinematics.getModuleAngle(p1, p2) + (PI/2.0f);

             SwerveState target = kinematics.getInverseKinematics(i, slew_vx, slew_vy, slew_omega);
             latest_targets[i] = target;

             float v1, v2;
             kinematics.mix(target, current_angle, v1, v2);
             motors.setVelocity(m1, v1);
             motors.setVelocity(m2, v2);
        }
    }
  }
  
  // --- DASHBOARD (10Hz) ---
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    
    Serial.print("SYS: "); 
    if (last_sys_state == STATE_DISABLED) Serial.print("DISABLED (IDLE)");
    else if (last_sys_state == STATE_STANDBY) Serial.print("STANDBY (HOLD)");
    else Serial.print("ACTIVE");
    
    bool healthy = true;
    for(int i=0; i<6; i++) { if(!motors.isConnected(i)) healthy = false; }
    if(!healthy) Serial.print(" [FAULT]");
    Serial.println();
    
    if (last_sys_state != STATE_DISABLED) {
        for (int i = 0; i < 3; i++) {
            int m1 = POD_MOTORS[i][0];
            int m2 = POD_MOTORS[i][1];
            
            float p1 = motors.getPosition(m1) - m1_offsets[i];
            float p2 = motors.getPosition(m2) - m2_offsets[i];
            float v1 = motors.getVelocity(m1);
            float v2 = motors.getVelocity(m2);

            float act_angle = kinematics.getModuleAngle(p1, p2) + (PI/2.0f);
            float act_speed = kinematics.getModuleDriveSpeed(v1, v2);
            
            float act_deg = act_angle * (180.0f/PI);
            float req_deg = latest_targets[i].angle * (180.0f/PI);

            // Normalize 0-360
            while(act_deg < 0) act_deg += 360; while(act_deg > 360) act_deg -= 360;
            while(req_deg < 0) req_deg += 360; while(req_deg > 360) req_deg -= 360;

            Serial.print("P"); Serial.print(i);
            // ANGLE: Actual / Requested
            Serial.print(": A/R "); Serial.print(act_deg, 0); Serial.print("/"); Serial.print(req_deg, 0);
            // VELOCITY: Actual / Requested
            Serial.print(" deg | V "); Serial.print(act_speed, 2); 
            Serial.print("/"); Serial.print(latest_targets[i].speed, 2);
            Serial.println(" m/s");
        }
        Serial.println();
    }
  }
}