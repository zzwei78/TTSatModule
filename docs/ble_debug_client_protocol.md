# BLE Debug Client Protocol

## Overview

TTSatModule supports 2 simultaneous BLE connections. The second connection is assigned the **DEBUG** role with restricted permissions, suitable for monitoring, diagnostics, and simulation control.

**Connection order**:
1. **1st connection** → PRIMARY role (full access, typically the phone app)
2. **2nd connection** → DEBUG role (restricted access, the control/debug tool)

**Role persistence**: If PRIMARY disconnects, DEBUG keeps its role. The next connection becomes PRIMARY.

---

## GATT Services

### Service Overview

| Service | UUID | DEBUG Access |
|---------|------|-------------|
| Voice | 0xABF0 | BLOCKED |
| AT Command | 0xABF2 | Read/Write/Notify |
| Log | 0xABF4 | Read/Notify |
| OTA | 0xABF8 | BLOCKED |
| System | 0xABFC | Limited commands |

---

### System Service (0xABFC) — Primary Control Channel

**Characteristics**:

| Characteristic | UUID | Properties | Description |
|---------------|------|------------|-------------|
| Control | 0xABFD | Read + Write + Notify | Send commands, receive responses |
| Info | 0xABFE | Read + Notify | System info (read-only) |
| Status | 0xABFF | Read + Notify | Status change notifications |

#### Command Packet Format

Write to Control characteristic (0xABFD):

```
Offset  Size  Field
0       1     Sequence number (0-255, wraps)
1       1     Command code
2       1     Parameter length (N)
3       N     Parameters
3+N     2     CRC16-CCITT
```

Maximum total packet size: 100 bytes (1+1+1+96+2).

#### Response Packet Format

Received via notification on Control characteristic (0xABFD):

```
Offset  Size  Field
0       1     Sequence number (echoed)
1       1     Command code (echoed)
2       1     Response code
3       1     Data length (N)
4       N     Response data
4+N     2     CRC16-CCITT
```

**Note**: For most commands, the response data format is `[resp_code][payload...]` where `resp_code` is at `data[0]` and payload starts at `data[1]`.

#### Response Codes

| Code | Name | Description |
|------|------|-------------|
| 0x00 | OK | Success |
| 0x01 | ERROR | General failure |
| 0x02 | INVALID_CMD | Unknown command |
| 0x03 | INVALID_PARAM | Bad parameters |
| 0x04 | SERVICE_NOT_FOUND | Service ID not recognized |
| 0x05 | SERVICE_ALREADY_RUNNING | Service already started |
| 0x06 | SERVICE_NOT_RUNNING | Service not started |

---

### AT Command Service (0xABF2)

| Characteristic | UUID | Properties |
|---------------|------|------------|
| AT Data | 0xABF3 | Read + Write + Notify |

**Usage**: Write AT command string to 0xABF3, receive response via notification on 0xABF3. Same access for both PRIMARY and DEBUG.

**Note**: When simulation is enabled, AT commands from GATT are intercepted by the simulation engine. DEBUG client can use this to verify simulation behavior.

---

### Log Service (0xABF4)

| Characteristic | UUID | Properties |
|---------------|------|------------|
| Log Data | 0xABF5 | Read + Notify |

**Usage**: Subscribe to notifications on 0xABF5 to receive real-time log output. Enable/configure via System Service commands.

**Log packet format**: `[level_byte][log_text][null_terminator]`

| Level | Value |
|-------|-------|
| DEBUG | 0 |
| INFO | 1 |
| WARN | 2 |
| ERROR | 3 |

---

## DEBUG Allowed Commands

### Query Commands (read device state)

#### 0x01 — GET_BATTERY_INFO

**Parameters**: None

**Response data** (after resp_code byte):

```
Offset  Size  Field
0       2     voltage_mv (uint16, little-endian)
2       2     current_ma (int16, little-endian, +charge/-discharge)
4       2     soc_percent (uint16, 0-100)
6       2     soh_percent (uint16, 0-100)
8       2     temperature_0_1k (uint16, 0.1K)
10      2     full_charge_capacity_mah (uint16)
12      2     remaining_capacity_mah (uint16)
14      1     charging (0/1)
15      1     full_charged (0/1)
```

Total: 16 bytes

---

#### 0x02 — GET_CHARGE_STATUS

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     is_charging (0/1)
1       1     is_full (0/1)
2       2     charge_voltage_mv (uint16, LE)
4       2     charge_current_ma (uint16, LE)
6       2     time_to_full_min (uint16, LE)
```

Total: 8 bytes

---

#### 0x03 — GET_TT_SIGNAL

**Status**: **DEPRECATED** — returns INVALID_CMD. Use AT Service to query `AT+CSQ` and `AT+CREG?` instead.

---

#### 0x04 — GET_BLE_TX_POWER

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     tx_power_dbm (int8, signed)
```

---

#### 0x30 — GET_SYSTEM_INFO

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       4     uptime_sec (uint32, LE)
4       4     free_heap (uint32, LE)
8       4     min_free_heap (uint32, LE)
12      1     cpu_freq_mhz
13      1     service_status (bitmask)
```

**service_status bitmask**:

| Bit | Service |
|-----|---------|
| 0 | OTA |
| 1 | LOG |
| 2 | AT |
| 3 | SPP/Voice data |
| 4 | Voice task |

---

#### 0x31 — GET_VERSION_INFO

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       16    firmware_version (null-terminated string)
16      24    software_version (null-terminated string)
40      16    manufacturer
56      16    model_number
72      8     hardware_revision
80      16    build_datetime (YYYY-MM-DD format)
```

Total: 96 bytes

---

#### 0x50 — GET_WORK_MODE_STATUS

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     work_mode
1       1     status_flags
```

---

#### 0x51 — GET_CHARGING_STATUS

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     status_flags (bitmask)
```

---

#### 0x52 — GET_WIRELESS_STATUS

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     wireless_status_flags
```

---

#### 0x54 — GET_VBUS_ADC

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       2     vbus_voltage_mv (uint16, LE)
2       2     vbus_current_ma (uint16, LE)
```

---

#### 0x55 — GET_NTC_DATA

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       2     ntc_temperature (raw ADC or 0.1K)
```

---

#### 0x60 — GET_TT_MODULE_STATE

**Parameters**: None

**Response data**:

```
Offset  Size  Field
0       1     state (enum: 0=off, 1=booting, 2=ready, 3=error, etc.)
1       2     voltage_mv (uint16, LE)
3       1     error_code
4       1     flags (bit0: force_on)
5       1     reserved
```

---

### Log Configuration Commands

#### 0x40 — SET_GLOBAL_LOG_LEVEL

**Parameters**:

```
Offset  Size  Field
0       1     level (0=DEBUG, 1=INFO, 2=WARN, 3=ERROR)
```

**Response**: resp_code only.

---

#### 0x41 — SET_MODULE_LOG_CONFIG

**Parameters**:

```
Offset  Size  Field
0       1     module_id (see module list, 0xFF=all)
1       1     log_level (0-4)
2       1     gatt_output (0=disable, 1=enable log output to GATT)
3       1     reserved
```

**Response**: resp_code only.

---

#### 0x42 — GET_MODULE_LOG_CONFIG

**Parameters**:

```
Offset  Size  Field
0       1     module_id (0xFF = get all modules)
```

**Response data**: Module log configuration (format depends on implementation).

---

#### 0x43 — SET_GATT_LOG_GLOBAL

**Parameters**:

```
Offset  Size  Field
0       1     enabled (0=disable, 1=enable)
```

**Response**: resp_code only. Enables/disables log output to GATT Log Service for all modules.

---

#### 0x44 — GET_GATT_LOG_STATE

**Parameters**: None

**Response data**: Current GATT log enable state.

---

#### 0x45 — GET_GATT_BUFFER_STATS

**Parameters**: None

**Response data**: Log buffer statistics (queue depth, drop count, etc.).

---

#### 0x46 — DISABLE_ALL_GATT_OUTPUT

**Parameters**: None

**Response**: resp_code only. Disables GATT log output for all modules at once.

---

### PowerBank Commands (IP5561)

#### 0x58 — SET_CHARGE_VOLTAGE

**Parameters**:

```
Offset  Size  Field
0       2     voltage_mv (uint16, LE, range: 4200-4400)
```

**Response**: resp_code only.

---

#### 0x59 — SET_CHARGE_CURRENT_9V

**Parameters**:

```
Offset  Size  Field
0       2     current_ma (uint16, LE)
```

**Response**: resp_code only.

---

#### 0x5A — SET_UV_THRESHOLD_9V

**Parameters**:

```
Offset  Size  Field
0       2     threshold_mv (uint16, LE)
```

**Response**: resp_code only.

---

#### 0x5C — SET_VBUS_OUTPUT_CURRENT_9V

**Parameters**:

```
Offset  Size  Field
0       2     current_ma (uint16, LE)
```

**Response**: resp_code only.

---

#### 0x5E — SET_WIRELESS_CHARGE

**Parameters**:

```
Offset  Size  Field
0       1     enabled (0=disable, 1=enable)
```

**Response data**: Echo of enabled state (1 byte).

---

### Simulation Control Commands

#### 0x77 — SIM_CTRL: Enable/Disable Simulation

**Parameters**:

```
Offset  Size  Field
0       1     enable (0=off, 1=on)
1       1     scenario (see below)
```

| Scenario | Value | Description |
|----------|-------|-------------|
| NORMAL | 0 | Call completes normally |
| BUSY | 1 | Remote party busy |
| NO_ANSWER | 2 | No answer (30s timeout) |
| NETWORK_DROP | 3 | Network drops during call (5-15s) |
| REJECT | 5 | Remote rejects after answering |

**Response**: resp_code only. Cannot change scenario while call is active.

---

#### 0x78 — SIM_INCOMING: Trigger Incoming Call

**Parameters**:

```
Offset  Size  Field
0       2     delay_ms (uint16, LE, 0=immediate)
```

**Response**: resp_code only. Returns ERROR if simulation disabled or call already active.

**Behavior**: Starts RING cycle (2s interval) to PRIMARY connection. Each RING also sends ^DSCI(incoming). Auto-timeout after 30s.

---

#### 0x79 — SIM_SET_NET: Set Simulated Network Parameters

**Parameters**:

```
Offset  Size  Field
0       1     csq (0-31, 99=unknown)
1       1     creg (0-5, see CREG values)
```

**Default**: CSQ=25, CREG=1 (registered, home).

**Response**: resp_code only.

---

### Limited System Control Commands

#### 0x22 — RESET_TT_MODULE

**Parameters**: None

**Response**: resp_code only. Hardware resets the Tiantong satellite module.

---

#### 0x24 — SET_USB_SWITCH

**Parameters**:

```
Offset  Size  Field
0       1     mode (0=USB to IP5561/charge, 1=USB to ESP32/comm)
```

**Response**: resp_code only.

---

## Commands BLOCKED for DEBUG

The following commands require PRIMARY role. DEBUG connections receive `BLE_ATT_ERR_INSUFFICIENT_AUTHOR` (0x8E):

| Code | Command | Reason |
|------|---------|--------|
| 0x05 | SET_BLE_TX_POWER | RF safety |
| 0x10 | SERVICE_START | Service lifecycle |
| 0x11 | SERVICE_STOP | Service lifecycle |
| 0x12 | SERVICE_STATUS | Service lifecycle |
| 0x20 | SYSTEM_REBOOT | System stability |
| 0x23 | REBOOT_TT_MCU | System stability |
| 0x25 | RESET_TT_HARDWARE | Hardware safety |
| 0x70 | SET_VOICE_FRAME_MODE | Call config |
| 0x71 | GET_VOICE_FRAME_MODE | Call config |
| 0x72 | TT_FORCE_ON | Power control |
| 0x73 | TT_FORCE_OFF | Power control |
| 0x74 | GET_SENSOR_STATUS | Sensor config |
| 0x75 | ENABLE_SENSOR_REPORT | Sensor config |
| 0x76 | DISABLE_SENSOR_REPORT | Sensor config |

---

## Typical Debug Client Workflow

### 1. Connect and Monitor

```
# 1. Connect as 2nd BLE device (auto-assigned DEBUG role)

# 2. Subscribe to Log Service (0xABF5) notifications
# 3. Enable GATT log output
Write 0xABFD → [seq, 0x43, 0x01, 0x01] + CRC  (SET_GATT_LOG_GLOBAL, enable=1)

# 4. Query device state
Write 0xABFD → [seq, 0x01, 0x00] + CRC  (GET_BATTERY_INFO)
Write 0xABFD → [seq, 0x30, 0x00] + CRC  (GET_SYSTEM_INFO)
Write 0xABFD → [seq, 0x60, 0x00] + CRC  (GET_TT_MODULE_STATE)
```

### 2. Run Simulation Test

```
# 5. Enable simulation with NORMAL scenario
Write 0xABFD → [seq, 0x77, 0x02, 0x01, 0x00] + CRC  (SIM_CTRL, enable=1, scenario=NORMAL)

# 6. PRIMARY app makes outgoing call (ATD10086;)
# Watch log output for call flow

# 7. Trigger incoming call
Write 0xABFD → [seq, 0x78, 0x02, 0x00, 0x00] + CRC  (SIM_INCOMING, delay=0)

# 8. PRIMARY app answers (ATA)
# Watch log output for call connected

# 9. Change scenario and test BUSY
Write 0xABFD → [seq, 0x77, 0x02, 0x01, 0x01] + CRC  (SIM_CTRL, enable=1, scenario=BUSY)
# PRIMARY app calls again → gets BUSY

# 10. Disable simulation when done
Write 0xABFD → [seq, 0x77, 0x02, 0x00, 0x00] + CRC  (SIM_CTRL, enable=0)
```

### 3. Power Management Debug

```
# Query battery
Write 0xABFD → [seq, 0x01, 0x00] + CRC  (GET_BATTERY_INFO)

# Query charge status
Write 0xABFD → [seq, 0x02, 0x00] + CRC  (GET_CHARGE_STATUS)

# Query VBUS voltage/current
Write 0xABFD → [seq, 0x54, 0x00] + CRC  (GET_VBUS_ADC)

# Adjust charge voltage
Write 0xABFD → [seq, 0x58, 0x02, 0x10, 0x10] + CRC  (SET_CHARGE_VOLTAGE, 4112mV)

# Enable wireless charging
Write 0xABFD → [seq, 0x5E, 0x01, 0x01] + CRC  (SET_WIRELESS_CHARGE, enable)
```

---

## CRC16-CCITT Calculation

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

CRC covers: seq + cmd + param_len + params (everything before the CRC field).
