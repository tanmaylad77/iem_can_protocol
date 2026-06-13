#include <Arduino.h>
#include <iem_can_protocol.h>

void setup() {
  Serial.begin(115200);

  IEMCanFrame frame = {
    IEM_CAN_ID_BMS_PACK_VI,
    IEM_CAN_FRAME_LEN,
    {0x14, 0x50, 0xDA, 0x2F, 0x00, 0x00, 0x00, 0x00}
  };

  IEMCanPackVI pack;
  if (iemCanUnpackPackVI(frame, pack)) {
    Serial.print("Pack voltage V: ");
    Serial.println(pack.pack_voltage_mv / 1000.0f, 3);
    Serial.print("Pack current A: ");
    Serial.println(pack.pack_current_ma / 1000.0f, 3);
  }
}

void loop() {
}
