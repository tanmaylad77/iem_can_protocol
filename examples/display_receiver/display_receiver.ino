#include <Arduino.h>
#include <iem_can_protocol.h>

static IEMCanMCDisplayState displayState;

static void redrawDisplay() {
  Serial.println("--- Driver Display ---");

  if (displayState.valid_flags & IEM_CAN_MC_DISPLAY_HAS_WHEEL_SPEED) {
    Serial.print("Speed rad/s: ");
    Serial.println(displayState.wheel_speed_rad_s, 1);
  } else {
    Serial.println("Speed rad/s: --");
  }

  if (displayState.valid_flags & IEM_CAN_MC_DISPLAY_HAS_COMMAND) {
    Serial.print("Throttle %: ");
    Serial.println(displayState.throttle_0_to_1 * 100.0f, 0);
    Serial.print("Requested current A: ");
    Serial.println(displayState.commanded_current_a, 1);
    Serial.print("Current limit A: ");
    Serial.println(displayState.current_limit_a, 1);
  } else {
    Serial.println("Throttle %: --");
    Serial.println("Requested current A: --");
    Serial.println("Current limit A: --");
  }
}

static void onCanFrameReceived(const IEMCanFrame &frame) {
  if (iemCanUpdateMCDisplayState(frame, displayState)) {
    redrawDisplay();
  }
}

void setup() {
  Serial.begin(115200);
  iemCanInitMCDisplayState(displayState);

  // Display firmware should initialize the Feather M4 CAN driver at IEM_CAN_BAUD
  // and the OLED here. Pass each received CAN frame to onCanFrameReceived().
  redrawDisplay();

  // Demo frames. Real display firmware receives these from the CAN driver.
  IEMCanFrame frame;
  iemCanPackMCCommand(40.0f, 55.0f, 0.63f, frame);
  onCanFrameReceived(frame);
  iemCanPackMCWheelSpeed(92.5f, frame);
  onCanFrameReceived(frame);
}

void loop() {
}
