#ifndef DIFFY_KINEMATICS_H
#define DIFFY_KINEMATICS_H

#include <Arduino.h>

// --- Robot Physical Constants ---
#define ROBOT_RADIUS  0.25f  // Meters (Center to Pod pivot)
#define GEAR_RATIO_N  2.5f  // Motor turns per Ring turn
#define GEAR_RATIO_L  1.0f   // Ring turns per Wheel turn
#define WHEEL_RADIUS  0.05f  // Meters

struct SwerveState {
  float speed; // m/s
  float angle; // radians
};

class DiffyKinematics {
  public:
    // Pod Locations: Top(0), Right(1), Left(2)
    const float MODULE_ANGLES[3] = {PI/2.0f, -PI/6.0f, -5.0f*PI/6.0f};

    // 1. INVERSE KINEMATICS
    SwerveState getInverseKinematics(int pod_index, float vx, float vy, float omega) {
       float rx = ROBOT_RADIUS * cos(MODULE_ANGLES[pod_index]);
       float ry = ROBOT_RADIUS * sin(MODULE_ANGLES[pod_index]);

       float v_rot_x = -omega * ry;
       float v_rot_y =  omega * rx;

       float total_vx = vx + v_rot_x;
       float total_vy = vy + v_rot_y;

       SwerveState target;
       target.speed = sqrt(total_vx*total_vx + total_vy*total_vy);
       target.angle = atan2(total_vy, total_vx);
       return target;
    }

    // 2. MIXER
    void mix(SwerveState target, float current_angle, float& motor1_vel, float& motor2_vel) {
       // A. Smart Angle Logic (DISABLED FOR DEBUGGING)
       // This forces the wheel to rotate to the exact target angle,
       // preventing "flipping" behavior so you can verify orientation.
       float angle_error = shortestSignedDist(target.angle, current_angle);
       
       /* OPTIMIZATION DISABLED
       if (abs(angle_error) > (PI/2.0f)) {
           target.speed = -target.speed; 
           angle_error = shortestSignedDist(target.angle + PI, current_angle);
       }
       */

       // B. P-Loop for Turning
       float Kp_steer = 15.0f; 
       float turn_vel_rads = angle_error * Kp_steer;
       
       // C. Drive Feedforward
       float drive_vel_rads = target.speed / WHEEL_RADIUS; 

       // D. Mixing
       float motor_drive_comp = drive_vel_rads * GEAR_RATIO_N * GEAR_RATIO_L;
       float motor_turn_comp  = turn_vel_rads * GEAR_RATIO_N;

       float m1_rads = motor_drive_comp + motor_turn_comp;
       float m2_rads = motor_drive_comp - motor_turn_comp;

       // E. Output (Rotations/s)
       motor1_vel = m1_rads / (2.0f * PI);
       motor2_vel = m2_rads / (2.0f * PI);
    }

    // Helper: Pos -> Angle
    float getModuleAngle(float pos_m1, float pos_m2) {
        float diff = pos_m1 - pos_m2; 
        float angle_rotations = diff / (2.0f * GEAR_RATIO_N); 
        return angle_rotations * 2.0f * PI; 
    }

    // NEW HELPER: Motor Vels -> Drive Speed (m/s)
    float getModuleDriveSpeed(float v1_rot, float v2_rot) {
        // v1, v2 in Rotations/s
        float v1_rad = v1_rot * 2.0f * PI;
        float v2_rad = v2_rot * 2.0f * PI;

        // Reverse the mixer:
        // m1 + m2 = 2 * drive
        float drive_component = (v1_rad + v2_rad) / 2.0f;
        
        // drive_component = wheel_rads * N * L
        float wheel_rads = drive_component / (GEAR_RATIO_N * GEAR_RATIO_L);
        return wheel_rads * WHEEL_RADIUS;
    }

    float shortestSignedDist(float target, float current) {
        float diff = target - current;
        while (diff <= -PI) diff += 2*PI;
        while (diff > PI) diff -= 2*PI;
        return diff;
    }
};

#endif