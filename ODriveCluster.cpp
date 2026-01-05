#include "ODriveCluster.h"

ODriveCluster::ODriveCluster() {
  for(int i=0; i<NUM_MOTORS; i++) {
    axes[i].connected = false;
    axes[i].pos_estimate = 0.0f;
    axes[i].vel_estimate = 0.0f;
  }
}

bool ODriveCluster::begin(int standby_pin, int boost_enable_pin) {
  pinMode(standby_pin, OUTPUT);
  digitalWrite(standby_pin, false);
  pinMode(boost_enable_pin, OUTPUT);
  digitalWrite(boost_enable_pin, true);

  return CAN.begin(ODRIVE_BAUD_RATE);
}

void ODriveCluster::processCAN() {
  while (CAN.parsePacket()) {
    uint8_t len = CAN.packetDlc();
    uint8_t data[8];
    for (int i = 0; i < len; i++) data[i] = CAN.read();

    uint32_t id = CAN.packetId();
    uint8_t node_id = id >> 5;
    uint8_t cmd_id  = id & 0x01F;

    if (node_id >= NUM_MOTORS) continue;

    if (cmd_id == CMD_HEARTBEAT || cmd_id == CMD_GET_ENCODER_ESTIMATES) {
       axes[node_id].connected = true;
       axes[node_id].last_msg_time = millis();
    }

    if (cmd_id == CMD_GET_ENCODER_ESTIMATES) {
      memcpy(&axes[node_id].pos_estimate, data, 4);
      memcpy(&axes[node_id].vel_estimate, data + 4, 4);
    }
  }
}

void ODriveCluster::waitForDiscovery(uint32_t timeout_ms) {
  Serial.print("Waiting for ODrives (Timeout: ");
  Serial.print(timeout_ms / 1000);
  Serial.println("s)...");

  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    processCAN(); 
    int count = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (axes[i].connected) count++;
    }
    if (count == NUM_MOTORS) break;
  }
  
  delay(1000); 

  Serial.println("Discovery Complete.");
  for(int i=0; i<NUM_MOTORS; i++) {
    Serial.print("Node "); Serial.print(i);
    Serial.println(axes[i].connected ? ": ONLINE" : ": MISSING");
  }
}

void ODriveCluster::sendMsg(uint8_t node_id, uint16_t cmd_id, void* data, uint8_t len) {
  uint32_t can_id = (node_id << 5) | cmd_id;
  CAN.beginPacket(can_id);
  CAN.write((uint8_t*)data, len);
  CAN.endPacket();
}

void ODriveCluster::enableClosedLoop(int axis_id) {
  if (!axes[axis_id].connected) return;
  uint32_t state = 8; // Closed Loop
  sendMsg(axis_id, CMD_SET_AXIS_STATE, &state, 4);
}

void ODriveCluster::setAxisState(int axis_id, int state) {
  if (!axes[axis_id].connected) return;
  uint32_t data = state;
  sendMsg(axis_id, CMD_SET_AXIS_STATE, &data, 4);
}

void ODriveCluster::setControlMode(int axis_id, int32_t control_mode, int32_t input_mode) {
  if (!axes[axis_id].connected) return;
  uint8_t buffer[8];
  memcpy(buffer, &control_mode, 4);
  memcpy(buffer + 4, &input_mode, 4);
  sendMsg(axis_id, CMD_SET_CONTROLLER_MODE, buffer, 8);
  delayMicroseconds(200);
}

void ODriveCluster::configurePID(int axis_id, float p, float i, float d) {
  if (!axes[axis_id].connected) return;
  uint8_t buffer[8] = {0};
  memcpy(buffer, &p, 4);
  sendMsg(axis_id, CMD_SET_POS_GAIN, buffer, 8);
  delayMicroseconds(200);
  memcpy(buffer, &d, 4); 
  memcpy(buffer + 4, &i, 4);
  sendMsg(axis_id, CMD_SET_VEL_GAINS, buffer, 8);
  delayMicroseconds(200);
}

void ODriveCluster::setPosition(int axis_id, float pos) {
  if (!axes[axis_id].connected) return;
  uint8_t buffer[8];
  int16_t vel_ff = 0;
  int16_t torque_ff = 0;
  memcpy(buffer, &pos, 4);
  memcpy(buffer + 4, &vel_ff, 2);
  memcpy(buffer + 6, &torque_ff, 2);
  sendMsg(axis_id, CMD_SET_INPUT_POS, buffer, 8);
}

void ODriveCluster::setVelocity(int axis_id, float vel) {
  if (!axes[axis_id].connected) return;
  uint8_t buffer[8];
  int16_t torque_ff = 0;
  memcpy(buffer, &vel, 4);
  memcpy(buffer + 4, &torque_ff, 2);
  sendMsg(axis_id, CMD_SET_INPUT_VEL, buffer, 6); 
}

bool ODriveCluster::isConnected(int axis_id) { return axes[axis_id].connected; }
float ODriveCluster::getPosition(int axis_id) { return axes[axis_id].pos_estimate; }
float ODriveCluster::getVelocity(int axis_id) { return axes[axis_id].vel_estimate; }

void ODriveCluster::printStatus() {
  Serial.println("--- System Status ---");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("Node "); Serial.print(i);
    Serial.print(": ");
    if (axes[i].connected) {
      Serial.print("P="); Serial.print(axes[i].pos_estimate, 2);
      Serial.print("  V="); Serial.print(axes[i].vel_estimate, 1);
    } else {
      Serial.print(" [OFFLINE]");
    }
    if (i % 2 != 0) Serial.println();
    else Serial.print("\t|\t");
  }
  Serial.println("\n---------------------");
}