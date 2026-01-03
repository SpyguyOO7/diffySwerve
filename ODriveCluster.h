#ifndef ODRIVE_CLUSTER_H
#define ODRIVE_CLUSTER_H

#include <Arduino.h>
#include <CANSAME5x.h>

// Configuration
#define NUM_MOTORS 6
#define ODRIVE_BAUD_RATE 500000 // Ensure this matches your ODrives!

// ODrive Command IDs
#define CMD_HEARTBEAT             0x001
#define CMD_SET_AXIS_STATE        0x007
#define CMD_GET_ENCODER_ESTIMATES 0x009
#define CMD_SET_INPUT_POS         0x00C
#define CMD_SET_POS_GAIN          0x01A
#define CMD_SET_VEL_GAINS         0x01B

struct ODriveAxis {
  float pos_estimate;
  float vel_estimate;
  bool  connected;
  uint32_t last_msg_time;
};

class ODriveCluster {
  public:
    ODriveCluster();

    // Initialization
    bool begin(int standby_pin, int boost_enable_pin);
    void waitForDiscovery(uint32_t timeout_ms);
    void enableClosedLoop(int axis_id);
    void configurePID(int axis_id, float p, float i, float d);

    // Runtime
    void processCAN(); // Call this frequently!
    void setPosition(int axis_id, float position);
    
    // Getters / Debug
    bool isConnected(int axis_id);
    float getPosition(int axis_id);
    float getVelocity(int axis_id);
    void printStatus();

  private:
    CANSAME5x CAN;
    ODriveAxis axes[NUM_MOTORS];

    // Internal helper to send raw CAN frames
    void sendMsg(uint8_t node_id, uint16_t cmd_id, void* data, uint8_t len);
};

#endif