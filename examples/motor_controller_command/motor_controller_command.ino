#include <Arduino.h>
#include <iem_can_protocol.h>

static void printFrame(const IEMCanFrame &frame) {
  Serial.print("TX ID: 0x");
  Serial.println(frame.id, HEX);
  Serial.print("TX data:");
  for (uint8_t i = 0; i < frame.len; i++) {
    Serial.print(" 0x");
    if (frame.data[i] < 16) Serial.print("0");
    Serial.print(frame.data[i], HEX);
  }
  Serial.println();
}

static void applyReceivedFrame(const IEMCanFrame &frame) {
  float commandedCurrentA;
  float throttle;
  float wheelSpeedRadS;

  if (iemCanUnpackMCWheelSpeedFloat(frame, wheelSpeedRadS)) {
    Serial.print("Wheel speed rad/s: ");
    Serial.println(wheelSpeedRadS, 3);
    return;
  }

  if (!iemCanUnpackMCCommandFloats(frame, commandedCurrentA, throttle)) {
    return;
  }

  Serial.print("Commanded current A: ");
  Serial.println(commandedCurrentA, 3);
  Serial.print("Throttle 0-1: ");
  Serial.println(throttle, 3);

  // MC firmware can pass commandedCurrentA and throttle into its control loop here.
}

void setup() {
  Serial.begin(115200);

  const float commandedCurrentA = 35.5f;
  const float throttle = 0.72f;

  IEMCanFrame frame;
  iemCanPackMCCommand(commandedCurrentA, throttle, frame);
  printFrame(frame);

  // In real firmware, this frame would arrive from the CAN driver receive path.
  applyReceivedFrame(frame);

  const float wheelSpeedRadS = 81.25f;
  iemCanPackMCWheelSpeed(wheelSpeedRadS, frame);
  printFrame(frame);
  applyReceivedFrame(frame);
}

void loop() {
}
