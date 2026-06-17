# IEM CAN Protocol

Shared Arduino-compatible CAN packing and unpacking helpers for STM32, ESP32, and SAMD51 firmware.

## Installation

For PlatformIO projects, pin a release tag in `platformio.ini`:

```ini
lib_deps =
  https://github.com/tanmaylad77/iem_can_protocol.git#v0.5.0
```

For Arduino IDE projects, install the library from a downloaded ZIP or add this repo under your sketchbook `libraries` folder.

Board-specific CAN setup stays in the consuming firmware. This library only defines the CAN IDs, fixed 8-byte payload layouts, and pack/unpack helpers.

## Protocol

The protocol uses 29-bit extended CAN IDs at an initial bus rate of 250 kbps. Payloads are always 8 bytes and little-endian. Voltages are sent in millivolts, current in milliamps, power in deciwatts, temperatures in deci-degrees C, and wheel speed in milliradians per second.

| ID | Frame | Payload |
| --- | --- | --- |
| `0x18B50000` | BMS heartbeat | version, node, state, safety, balancing, flags, uptime_s |
| `0x18B50001` | Pack voltage/current | pack_mV, pack_mA |
| `0x18B50002` | Pack power | pack_dW, SOC %, SOH % (`255` = unknown) |
| `0x18B50003` | Cells 1-2 | cell_1_mV, cell_2_mV |
| `0x18B50004` | Cells 3-4 | cell_3_mV, cell_4_mV |
| `0x18B50005` | Cell summary | cell_5_mV, min_mV, max_mV, spread_mV |
| `0x18B50006` | Temperatures | temp_1_dC, temp_2_dC, temp_3_dC |
| `0x18B50007` | Safety | safety_status, last_fault_status, state, flags, last_fault_ms |
| `0x18B50008` | Balancing | balancing_status, limits_revision, cell_count, protocol_version |
| `0x18B50009` | Cell limits | cell_OV_mV, cell_UV_mV, balance_threshold_mV, balance_tolerance_mV |
| `0x18B5000A` | Pack limits | pack_OV_mV, pack_UV_mV |
| `0x18B5000B` | Current/temp limits | pack_OC_mA, temp_high_dC, temp_low_dC |
| `0x18C00000` | MC command | commanded_current_mA, throttle_raw, current_limit_mA |
| `0x18C00001` | MC wheel speed | wheel_speed_mrad_s |

## Motor Controller Command

The motor-controller command helper accepts values in the units normally used by controller firmware:

```cpp
IEMCanFrame frame;
iemCanPackMCCommand(commandedCurrentA, currentLimitA, throttlePosition, frame);
```

`commandedCurrentA` is a signed float in amps. `currentLimitA` is a positive float in amps and is clamped to the payload range `0-65.535 A`. `throttlePosition` is a float from `0.0` to `1.0`; the helper clamps it to the CAN payload range `0-255`.

On the receiving side, use either the raw payload:

```cpp
IEMCanMCCommand command;
if (iemCanUnpackMCCommand(frame, command)) {
  float currentA = iemCanMCCommandCurrentA(command);
  float currentLimitA = iemCanMCCommandCurrentLimitA(command);
  float throttle = iemCanMCCommandThrottle(command);
}
```

or unpack directly to floats:

```cpp
float currentA;
float currentLimitA;
float throttle;
if (iemCanUnpackMCCommandFloats(frame, currentA, currentLimitA, throttle)) {
  // Apply command to the motor controller.
}
```

## Motor Controller Wheel Speed

Wheel speed helpers accept and return rad/s in firmware code:

```cpp
IEMCanFrame frame;
iemCanPackMCWheelSpeed(wheelSpeedRadS, frame);
```

On the receiving side:

```cpp
float wheelSpeedRadS;
if (iemCanUnpackMCWheelSpeedFloat(frame, wheelSpeedRadS)) {
  // Use wheelSpeedRadS in control, logging, or telemetry.
}
```

The CAN payload stores the value as signed milliradians per second, so negative speeds can be represented if the motor-controller convention needs them.

## Driver Display Receiver

For an Adafruit Feather M4 CAN with an OLED display, keep the CAN driver and OLED drawing code in the display firmware, and use this library only for decoding. The Feather M4 CAN has built-in CAN hardware and transceiver support, and Adafruit documents Arduino/CircuitPython support for reading and writing CAN packets.

The display firmware can maintain a cached state like this:

```cpp
IEMCanMCDisplayState displayState;

void setup() {
  iemCanInitMCDisplayState(displayState);
  // Init CAN driver at IEM_CAN_BAUD and init OLED here.
}

void onCanFrame(const IEMCanFrame &frame) {
  if (iemCanUpdateMCDisplayState(frame, displayState)) {
    // Redraw OLED from displayState.throttle_0_to_1,
    // displayState.commanded_current_a, displayState.current_limit_a,
    // and displayState.wheel_speed_rad_s.
  }
}
```

The helper sets `IEM_CAN_MC_DISPLAY_HAS_COMMAND` once a throttle/current command has been received and `IEM_CAN_MC_DISPLAY_HAS_WHEEL_SPEED` once wheel speed has been received. This lets the OLED show placeholders until real CAN data has arrived.

Keep this library platform-neutral. Board-specific CAN driver setup belongs in the consuming firmware, not in this protocol layer.
