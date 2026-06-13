#include "iem_can_protocol.h"

#include <math.h>
#include <string.h>

static void initFrame(IEMCanFrame &frame, uint32_t id) {
    frame.id = id;
    frame.len = IEM_CAN_FRAME_LEN;
    memset(frame.data, 0, sizeof(frame.data));
}

static int32_t roundScaled(float value, float scale) {
    // Non-finite ADC-derived values should not poison the bus; encode them as 0.
    if (!isfinite(value)) return 0;
    float scaled = value * scale;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint16_t clampU16(int32_t value) {
    if (value < 0) return 0;
    if (value > 65535) return 65535;
    return (uint16_t)value;
}

static int16_t clampI16(int32_t value) {
    if (value < -32768) return -32768;
    if (value > 32767) return 32767;
    return (int16_t)value;
}

static void putU16(uint8_t *data, uint8_t index, uint16_t value) {
    // CAN payloads are explicitly little-endian so receivers do not depend on MCU endianness.
    data[index] = (uint8_t)(value & 0xFF);
    data[index + 1] = (uint8_t)((value >> 8) & 0xFF);
}

static uint16_t getU16(const uint8_t *data, uint8_t index) {
    return (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
}

static void putI16(uint8_t *data, uint8_t index, int16_t value) {
    putU16(data, index, (uint16_t)value);
}

static int16_t getI16(const uint8_t *data, uint8_t index) {
    return (int16_t)getU16(data, index);
}

static void putU32(uint8_t *data, uint8_t index, uint32_t value) {
    data[index] = (uint8_t)(value & 0xFF);
    data[index + 1] = (uint8_t)((value >> 8) & 0xFF);
    data[index + 2] = (uint8_t)((value >> 16) & 0xFF);
    data[index + 3] = (uint8_t)((value >> 24) & 0xFF);
}

static uint32_t getU32(const uint8_t *data, uint8_t index) {
    return (uint32_t)data[index] |
           ((uint32_t)data[index + 1] << 8) |
           ((uint32_t)data[index + 2] << 16) |
           ((uint32_t)data[index + 3] << 24);
}

static void putI32(uint8_t *data, uint8_t index, int32_t value) {
    putU32(data, index, (uint32_t)value);
}

static int32_t getI32(const uint8_t *data, uint8_t index) {
    return (int32_t)getU32(data, index);
}

static bool isFrame(const IEMCanFrame &frame, uint32_t id) {
    return frame.id == id && frame.len == IEM_CAN_FRAME_LEN;
}

uint8_t iemCanMakeBMSFlags(bool latched, bool estop_pressed, uint8_t safety_status) {
    uint8_t flags = 0;
    if (latched) flags |= IEM_CAN_BMS_FLAG_LATCHED;
    if (estop_pressed) flags |= IEM_CAN_BMS_FLAG_ESTOP;
    if (safety_status != 0) flags |= IEM_CAN_BMS_FLAG_FAULT;
    return flags;
}

uint8_t iemCanMakeBMSState(bool latched, bool estop_pressed, uint8_t safety_status) {
    if (safety_status != 0) return IEM_CAN_BMS_STATE_FAULT;
    if (estop_pressed) return IEM_CAN_BMS_STATE_ESTOP;
    if (latched) return IEM_CAN_BMS_STATE_RUN;
    return IEM_CAN_BMS_STATE_IDLE;
}

void iemCanPackHeartbeat(const IEMCanHeartbeat &payload, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_HEARTBEAT);
    frame.data[0] = payload.protocol_version;
    frame.data[1] = payload.node_id;
    frame.data[2] = payload.state;
    frame.data[3] = payload.safety_status;
    frame.data[4] = payload.balancing_status;
    frame.data[5] = payload.flags;
    putU16(frame.data, 6, payload.uptime_s);
}

bool iemCanUnpackHeartbeat(const IEMCanFrame &frame, IEMCanHeartbeat &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_HEARTBEAT)) return false;
    payload.protocol_version = frame.data[0];
    payload.node_id = frame.data[1];
    payload.state = frame.data[2];
    payload.safety_status = frame.data[3];
    payload.balancing_status = frame.data[4];
    payload.flags = frame.data[5];
    payload.uptime_s = getU16(frame.data, 6);
    return true;
}

void iemCanPackPackVI(float pack_voltage_v, float pack_current_a, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_PACK_VI);
    putU16(frame.data, 0, clampU16(roundScaled(pack_voltage_v, 1000.0f)));
    putI32(frame.data, 2, roundScaled(pack_current_a, 1000.0f));
}

bool iemCanUnpackPackVI(const IEMCanFrame &frame, IEMCanPackVI &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_PACK_VI)) return false;
    payload.pack_voltage_mv = getU16(frame.data, 0);
    payload.pack_current_ma = getI32(frame.data, 2);
    return true;
}

void iemCanPackPackPower(float pack_power_w, uint8_t soc_percent, uint8_t soh_percent, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_PACK_POWER);
    putI32(frame.data, 0, roundScaled(pack_power_w, 10.0f));
    frame.data[4] = soc_percent;
    frame.data[5] = soh_percent;
}

bool iemCanUnpackPackPower(const IEMCanFrame &frame, IEMCanPackPower &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_PACK_POWER)) return false;
    payload.pack_power_dw = getI32(frame.data, 0);
    payload.soc_percent = frame.data[4];
    payload.soh_percent = frame.data[5];
    return true;
}

void iemCanPackCellPair(uint32_t id, float cell_a_v, float cell_b_v, IEMCanFrame &frame) {
    // The caller chooses CELLS_1_2 or CELLS_3_4; both share the same payload shape.
    initFrame(frame, id);
    putU16(frame.data, 0, clampU16(roundScaled(cell_a_v, 1000.0f)));
    putU16(frame.data, 2, clampU16(roundScaled(cell_b_v, 1000.0f)));
}

bool iemCanUnpackCellPair(const IEMCanFrame &frame, IEMCanCellPair &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_CELLS_1_2) && !isFrame(frame, IEM_CAN_ID_BMS_CELLS_3_4)) return false;
    payload.cell_a_mv = getU16(frame.data, 0);
    payload.cell_b_mv = getU16(frame.data, 2);
    return true;
}

void iemCanPackCellSummary(float cell_5_v, float min_cell_v, float max_cell_v, IEMCanFrame &frame) {
    uint16_t min_mv = clampU16(roundScaled(min_cell_v, 1000.0f));
    uint16_t max_mv = clampU16(roundScaled(max_cell_v, 1000.0f));
    uint16_t spread_mv = max_mv >= min_mv ? (uint16_t)(max_mv - min_mv) : 0;

    initFrame(frame, IEM_CAN_ID_BMS_CELL_SUMMARY);
    putU16(frame.data, 0, clampU16(roundScaled(cell_5_v, 1000.0f)));
    putU16(frame.data, 2, min_mv);
    putU16(frame.data, 4, max_mv);
    putU16(frame.data, 6, spread_mv);
}

bool iemCanUnpackCellSummary(const IEMCanFrame &frame, IEMCanCellSummary &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_CELL_SUMMARY)) return false;
    payload.cell_5_mv = getU16(frame.data, 0);
    payload.min_cell_mv = getU16(frame.data, 2);
    payload.max_cell_mv = getU16(frame.data, 4);
    payload.spread_mv = getU16(frame.data, 6);
    return true;
}

void iemCanPackTemperatures(float temp_1_c, float temp_2_c, float temp_3_c, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_TEMPERATURES);
    putI16(frame.data, 0, clampI16(roundScaled(temp_1_c, 10.0f)));
    putI16(frame.data, 2, clampI16(roundScaled(temp_2_c, 10.0f)));
    putI16(frame.data, 4, clampI16(roundScaled(temp_3_c, 10.0f)));
}

bool iemCanUnpackTemperatures(const IEMCanFrame &frame, IEMCanTemperatures &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_TEMPERATURES)) return false;
    payload.temp_1_dc = getI16(frame.data, 0);
    payload.temp_2_dc = getI16(frame.data, 2);
    payload.temp_3_dc = getI16(frame.data, 4);
    return true;
}

void iemCanPackSafety(const IEMCanSafety &payload, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_SAFETY);
    frame.data[0] = payload.safety_status;
    frame.data[1] = payload.last_fault_status;
    frame.data[2] = payload.state;
    frame.data[3] = payload.flags;
    putU32(frame.data, 4, payload.last_fault_ms);
}

bool iemCanUnpackSafety(const IEMCanFrame &frame, IEMCanSafety &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_SAFETY)) return false;
    payload.safety_status = frame.data[0];
    payload.last_fault_status = frame.data[1];
    payload.state = frame.data[2];
    payload.flags = frame.data[3];
    payload.last_fault_ms = getU32(frame.data, 4);
    return true;
}

void iemCanPackBalancing(const IEMCanBalancing &payload, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_BALANCING);
    frame.data[0] = payload.balancing_status;
    frame.data[1] = payload.limits_revision;
    frame.data[2] = payload.cell_count;
    frame.data[3] = payload.protocol_version;
}

bool iemCanUnpackBalancing(const IEMCanFrame &frame, IEMCanBalancing &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_BALANCING)) return false;
    payload.balancing_status = frame.data[0];
    payload.limits_revision = frame.data[1];
    payload.cell_count = frame.data[2];
    payload.protocol_version = frame.data[3];
    return true;
}

void iemCanPackCellLimits(float cell_ov_v, float cell_uv_v, float balance_threshold_v, float balance_tolerance_v, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_LIMITS_CELL);
    putU16(frame.data, 0, clampU16(roundScaled(cell_ov_v, 1000.0f)));
    putU16(frame.data, 2, clampU16(roundScaled(cell_uv_v, 1000.0f)));
    putU16(frame.data, 4, clampU16(roundScaled(balance_threshold_v, 1000.0f)));
    putU16(frame.data, 6, clampU16(roundScaled(balance_tolerance_v, 1000.0f)));
}

bool iemCanUnpackCellLimits(const IEMCanFrame &frame, IEMCanCellLimits &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_LIMITS_CELL)) return false;
    payload.cell_ov_mv = getU16(frame.data, 0);
    payload.cell_uv_mv = getU16(frame.data, 2);
    payload.balance_threshold_mv = getU16(frame.data, 4);
    payload.balance_tolerance_mv = getU16(frame.data, 6);
    return true;
}

void iemCanPackPackLimits(float pack_ov_v, float pack_uv_v, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_LIMITS_PACK);
    putU16(frame.data, 0, clampU16(roundScaled(pack_ov_v, 1000.0f)));
    putU16(frame.data, 2, clampU16(roundScaled(pack_uv_v, 1000.0f)));
}

bool iemCanUnpackPackLimits(const IEMCanFrame &frame, IEMCanPackLimits &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_LIMITS_PACK)) return false;
    payload.pack_ov_mv = getU16(frame.data, 0);
    payload.pack_uv_mv = getU16(frame.data, 2);
    return true;
}

void iemCanPackCurrentTempLimits(float pack_oc_a, float temp_high_c, float temp_low_c, IEMCanFrame &frame) {
    initFrame(frame, IEM_CAN_ID_BMS_LIMITS_CURRENT_TEMP);
    putI32(frame.data, 0, roundScaled(pack_oc_a, 1000.0f));
    putI16(frame.data, 4, clampI16(roundScaled(temp_high_c, 10.0f)));
    putI16(frame.data, 6, clampI16(roundScaled(temp_low_c, 10.0f)));
}

bool iemCanUnpackCurrentTempLimits(const IEMCanFrame &frame, IEMCanCurrentTempLimits &payload) {
    if (!isFrame(frame, IEM_CAN_ID_BMS_LIMITS_CURRENT_TEMP)) return false;
    payload.pack_oc_ma = getI32(frame.data, 0);
    payload.temp_high_dc = getI16(frame.data, 4);
    payload.temp_low_dc = getI16(frame.data, 6);
    return true;
}
