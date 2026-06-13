# IEM CAN Protocol

Shared Arduino-compatible CAN packing and unpacking helpers for STM32, ESP32, and SAMD51 firmware.

## Installation

For PlatformIO projects, pin a release tag in `platformio.ini`:

```ini
lib_deps =
  https://github.com/tanmaylad77/iem_can_protocol.git#v0.2.0
```

For Arduino IDE projects, install the library from a downloaded ZIP or add this repo under your sketchbook `libraries` folder.

Board-specific CAN setup stays in the consuming firmware. This library only defines the CAN IDs, fixed 8-byte payload layouts, and pack/unpack helpers.

## Protocol

The protocol uses 29-bit extended CAN IDs at an initial bus rate of 250 kbps. Payloads are always 8 bytes and little-endian. Voltages are sent in millivolts, current in milliamps, power in deciwatts, and temperatures in deci-degrees C.

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
| `0x18C00000` | MC command | commanded_current_mA, throttle_raw |

## Motor Controller Command

The motor-controller command helper accepts values in the units normally used by controller firmware:

```cpp
IEMCanFrame frame;
iemCanPackMCCommand(commandedCurrentA, throttlePosition, frame);
```

`commandedCurrentA` is a signed float in amps. `throttlePosition` is a float from `0.0` to `1.0`; the helper clamps it to the CAN payload range `0-255`.

On the receiving side, use either the raw payload:

```cpp
IEMCanMCCommand command;
if (iemCanUnpackMCCommand(frame, command)) {
  float currentA = iemCanMCCommandCurrentA(command);
  float throttle = iemCanMCCommandThrottle(command);
}
```

or unpack directly to floats:

```cpp
float currentA;
float throttle;
if (iemCanUnpackMCCommandFloats(frame, currentA, throttle)) {
  // Apply command to the motor controller.
}
```

Keep this library platform-neutral. Board-specific CAN driver setup belongs in the consuming firmware, not in this protocol layer.
