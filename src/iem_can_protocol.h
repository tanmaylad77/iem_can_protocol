#ifndef IEM_CAN_PROTOCOL_H
#define IEM_CAN_PROTOCOL_H

#include <stdint.h>

#define IEM_CAN_PROTOCOL_VERSION 1
#define IEM_CAN_LIMITS_REVISION 1
#define IEM_CAN_BAUD 250000UL
#define IEM_CAN_FRAME_LEN 8
#define IEM_CAN_NODE_BMS 1

// Payload scaling used across all frames:
// voltage = mV, current = mA, power = deciwatts, temperature = deci-degrees C.
// Extended 29-bit IDs. Keep these definitions shared by all Arduino targets
// so STM32, ESP32, SAMD51, and motor controller firmware agree on the wire format.
#define IEM_CAN_BMS_BASE_ID             0x18B50000UL
#define IEM_CAN_ID_BMS_HEARTBEAT        (IEM_CAN_BMS_BASE_ID + 0x00UL)
#define IEM_CAN_ID_BMS_PACK_VI          (IEM_CAN_BMS_BASE_ID + 0x01UL)
#define IEM_CAN_ID_BMS_PACK_POWER       (IEM_CAN_BMS_BASE_ID + 0x02UL)
#define IEM_CAN_ID_BMS_CELLS_1_2        (IEM_CAN_BMS_BASE_ID + 0x03UL)
#define IEM_CAN_ID_BMS_CELLS_3_4        (IEM_CAN_BMS_BASE_ID + 0x04UL)
#define IEM_CAN_ID_BMS_CELL_SUMMARY     (IEM_CAN_BMS_BASE_ID + 0x05UL)
#define IEM_CAN_ID_BMS_TEMPERATURES     (IEM_CAN_BMS_BASE_ID + 0x06UL)
#define IEM_CAN_ID_BMS_SAFETY           (IEM_CAN_BMS_BASE_ID + 0x07UL)
#define IEM_CAN_ID_BMS_BALANCING        (IEM_CAN_BMS_BASE_ID + 0x08UL)
#define IEM_CAN_ID_BMS_LIMITS_CELL      (IEM_CAN_BMS_BASE_ID + 0x09UL)
#define IEM_CAN_ID_BMS_LIMITS_PACK      (IEM_CAN_BMS_BASE_ID + 0x0AUL)
#define IEM_CAN_ID_BMS_LIMITS_CURRENT_TEMP (IEM_CAN_BMS_BASE_ID + 0x0BUL)

enum IEMCanBMSState : uint8_t {
    IEM_CAN_BMS_STATE_IDLE = 0,
    IEM_CAN_BMS_STATE_RUN = 1,
    IEM_CAN_BMS_STATE_ESTOP = 2,
    IEM_CAN_BMS_STATE_FAULT = 3
};

enum IEMCanBMSFlags : uint8_t {
    IEM_CAN_BMS_FLAG_LATCHED = 1 << 0,
    IEM_CAN_BMS_FLAG_ESTOP = 1 << 1,
    IEM_CAN_BMS_FLAG_FAULT = 1 << 2
};

struct IEMCanFrame {
    uint32_t id;
    uint8_t len;
    uint8_t data[IEM_CAN_FRAME_LEN];
};

struct IEMCanHeartbeat {
    uint8_t protocol_version;
    uint8_t node_id;
    uint8_t state;
    uint8_t safety_status;
    uint8_t balancing_status;
    uint8_t flags;
    uint16_t uptime_s;
};

struct IEMCanPackVI {
    uint16_t pack_voltage_mv;
    int32_t pack_current_ma;
};

struct IEMCanPackPower {
    int32_t pack_power_dw;
    uint8_t soc_percent;
    uint8_t soh_percent;
};

struct IEMCanCellPair {
    uint16_t cell_a_mv;
    uint16_t cell_b_mv;
};

struct IEMCanCellSummary {
    uint16_t cell_5_mv;
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;
    uint16_t spread_mv;
};

struct IEMCanTemperatures {
    int16_t temp_1_dc;
    int16_t temp_2_dc;
    int16_t temp_3_dc;
};

struct IEMCanSafety {
    uint8_t safety_status;
    uint8_t last_fault_status;
    uint8_t state;
    uint8_t flags;
    uint32_t last_fault_ms;
};

struct IEMCanBalancing {
    uint8_t balancing_status;
    uint8_t limits_revision;
    uint8_t cell_count;
    uint8_t protocol_version;
};

struct IEMCanCellLimits {
    uint16_t cell_ov_mv;
    uint16_t cell_uv_mv;
    uint16_t balance_threshold_mv;
    uint16_t balance_tolerance_mv;
};

struct IEMCanPackLimits {
    uint16_t pack_ov_mv;
    uint16_t pack_uv_mv;
};

struct IEMCanCurrentTempLimits {
    int32_t pack_oc_ma;
    int16_t temp_high_dc;
    int16_t temp_low_dc;
};

uint8_t iemCanMakeBMSFlags(bool latched, bool estop_pressed, uint8_t safety_status);
uint8_t iemCanMakeBMSState(bool latched, bool estop_pressed, uint8_t safety_status);

void iemCanPackHeartbeat(const IEMCanHeartbeat &payload, IEMCanFrame &frame);
bool iemCanUnpackHeartbeat(const IEMCanFrame &frame, IEMCanHeartbeat &payload);

void iemCanPackPackVI(float pack_voltage_v, float pack_current_a, IEMCanFrame &frame);
bool iemCanUnpackPackVI(const IEMCanFrame &frame, IEMCanPackVI &payload);

void iemCanPackPackPower(float pack_power_w, uint8_t soc_percent, uint8_t soh_percent, IEMCanFrame &frame);
bool iemCanUnpackPackPower(const IEMCanFrame &frame, IEMCanPackPower &payload);

void iemCanPackCellPair(uint32_t id, float cell_a_v, float cell_b_v, IEMCanFrame &frame);
bool iemCanUnpackCellPair(const IEMCanFrame &frame, IEMCanCellPair &payload);

void iemCanPackCellSummary(float cell_5_v, float min_cell_v, float max_cell_v, IEMCanFrame &frame);
bool iemCanUnpackCellSummary(const IEMCanFrame &frame, IEMCanCellSummary &payload);

void iemCanPackTemperatures(float temp_1_c, float temp_2_c, float temp_3_c, IEMCanFrame &frame);
bool iemCanUnpackTemperatures(const IEMCanFrame &frame, IEMCanTemperatures &payload);

void iemCanPackSafety(const IEMCanSafety &payload, IEMCanFrame &frame);
bool iemCanUnpackSafety(const IEMCanFrame &frame, IEMCanSafety &payload);

void iemCanPackBalancing(const IEMCanBalancing &payload, IEMCanFrame &frame);
bool iemCanUnpackBalancing(const IEMCanFrame &frame, IEMCanBalancing &payload);

void iemCanPackCellLimits(float cell_ov_v, float cell_uv_v, float balance_threshold_v, float balance_tolerance_v, IEMCanFrame &frame);
bool iemCanUnpackCellLimits(const IEMCanFrame &frame, IEMCanCellLimits &payload);

void iemCanPackPackLimits(float pack_ov_v, float pack_uv_v, IEMCanFrame &frame);
bool iemCanUnpackPackLimits(const IEMCanFrame &frame, IEMCanPackLimits &payload);

void iemCanPackCurrentTempLimits(float pack_oc_a, float temp_high_c, float temp_low_c, IEMCanFrame &frame);
bool iemCanUnpackCurrentTempLimits(const IEMCanFrame &frame, IEMCanCurrentTempLimits &payload);

#endif // IEM_CAN_PROTOCOL_H
