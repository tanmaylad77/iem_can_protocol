#include <Arduino.h>
#include <iem_can_protocol.h>

void setup() {
  Serial.begin(115200);

  IEMCanFrame frame;
  iemCanPackPackVI(20.5f, 12.25f, frame);

  Serial.print("ID: 0x");
  Serial.println(frame.id, HEX);
  Serial.print("Data:");
  for (uint8_t i = 0; i < frame.len; i++) {
    Serial.print(" 0x");
    if (frame.data[i] < 16) Serial.print("0");
    Serial.print(frame.data[i], HEX);
  }
  Serial.println();
}

void loop() {
}
