# Satellite Call Simulation Protocol

## Overview

The satellite call simulation system allows testing call flows without a real satellite network. It intercepts GATT-path AT commands and generates realistic responses/URCs. LOCAL AT commands are not affected.

**Test phone number**: `10086` (fixed)

**Default state**: Disabled at boot, must be enabled via BLE system command.

## BLE System Commands

All commands use the System Service UUID `0xABFC`, Control Characteristic `0xABFD`.

### Packet Format

**Command packet**:
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Sequence number |
| 1 | 1 | Command code |
| 2 | 1 | Parameter length |
| 3 | N | Parameters |
| 3+N | 2 | CRC16-CCITT |

**Response packet**:
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Sequence number (echoed) |
| 1 | 1 | Command code (echoed) |
| 2 | 1 | Response code |
| 3 | 1 | Data length |
| 4 | N | Response data |
| 4+N | 2 | CRC16-CCITT |

**Response codes**:
| Code | Meaning |
|------|---------|
| 0x00 | OK |
| 0x01 | ERROR |
| 0x02 | INVALID_CMD |
| 0x03 | INVALID_PARAM |

---

### 0x77 - SIM_CTRL: Enable/Disable Simulation

**Parameters**:
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Enable (1=on, 0=off) |
| 1 | 1 | Scenario (see below) |

**Scenario values**:
| Value | Name | Description |
|-------|------|-------------|
| 0 | NORMAL | Call completes normally (answered) |
| 1 | BUSY | Remote party is busy |
| 2 | NO_ANSWER | No answer (30s timeout) |
| 3 | NETWORK_DROP | Network drops during call (5-15s random) |
| 5 | REJECT | Remote party rejects after answering |

**Response data**: Empty on success.

**Example**: Enable simulation with NORMAL scenario
```
CMD:  seq=0x01, cmd=0x77, params=[0x01, 0x00]
RESP: seq=0x01, cmd=0x77, resp=0x00, data=[]
```

**Example**: Enable simulation with BUSY scenario
```
CMD:  seq=0x02, cmd=0x77, params=[0x01, 0x01]
RESP: seq=0x02, cmd=0x77, resp=0x00, data=[]
```

**Example**: Disable simulation
```
CMD:  seq=0x03, cmd=0x77, params=[0x00, 0x00]
RESP: seq=0x03, cmd=0x77, resp=0x00, data=[]
```

---

### 0x78 - SIM_INCOMING: Trigger Incoming Call

**Parameters**:
| Offset | Size | Field |
|--------|------|-------|
| 0 | 2 | Delay before first RING (little-endian, ms, 0=immediate) |

**Response data**: Empty on success. Returns ERROR if simulation is disabled or a call is already active.

**Behavior**: Starts a periodic RING cycle (2s interval). Each cycle sends RING + ^DSCI(incoming). Auto-times out after 30s if not answered.

**Example**: Trigger incoming call immediately
```
CMD:  seq=0x01, cmd=0x78, params=[0x00, 0x00]
RESP: seq=0x01, cmd=0x78, resp=0x00, data=[]
```

**Example**: Trigger incoming call after 5 seconds
```
CMD:  seq=0x02, cmd=0x78, params=[0x88, 0x13]  // 5000ms = 0x1388
RESP: seq=0x02, cmd=0x78, resp=0x00, data=[]
```

---

### 0x79 - SIM_SET_NET: Set Simulated Network Parameters

**Parameters**:
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | CSQ signal quality (0-31, 99=unknown) |
| 1 | 1 | CREG registration status (0-5) |

**CREG values**:
| Value | Meaning |
|-------|---------|
| 0 | Not registered, not searching |
| 1 | Registered, home network |
| 2 | Not registered, searching |
| 3 | Registration denied |
| 4 | Unknown |
| 5 | Registered, roaming |

**Default values**: CSQ=25, CREG=1 (registered, home network)

**Response data**: Empty on success.

**Example**: Set good signal, registered
```
CMD:  seq=0x01, cmd=0x79, params=[0x19, 0x01]  // CSQ=25, CREG=1
RESP: seq=0x01, cmd=0x79, resp=0x00, data=[]
```

**Example**: Set no signal, searching
```
CMD:  seq=0x02, cmd=0x79, params=[0x63, 0x02]  // CSQ=99, CREG=2
RESP: seq=0x02, cmd=0x79, resp=0x00, data=[]
```

---

### 0x7A - SIM_GET_STATE: Query Simulation State

**Parameters**: None

**Response data** (3 bytes):
| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | enabled | 0=disabled, 1=enabled |
| 1 | 1 | scenario | 0=NORMAL, 1=BUSY, 2=NO_ANSWER, 3=NETWORK_DROP, 5=REJECT |
| 2 | 1 | call_state | 0=IDLE, 1=DIALING, 2=ALERTING, 3=ACTIVE, 4=INCOMING, 5=DISCONNECTING |

**Example**: Query state when simulation is active with NORMAL scenario, call connected
```
CMD:  seq=0x01, cmd=0x7A, params=[]
RESP: seq=0x01, cmd=0x7A, resp=0x00, data=[0x01, 0x00, 0x03]
// enabled=1, scenario=NORMAL(0), call_state=ACTIVE(3)
```

**Example**: Query state when simulation is disabled
```
CMD:  seq=0x02, cmd=0x7A, params=[]
RESP: seq=0x02, cmd=0x7A, resp=0x00, data=[0x00, 0x00, 0x00]
// enabled=0, scenario=NORMAL(0), call_state=IDLE(0)
```

**Call state values**:
| Value | State |
|-------|-------|
| 0 | IDLE |
| 1 | DIALING |
| 2 | ALERTING |
| 3 | ACTIVE |
| 4 | INCOMING |
| 5 | DISCONNECTING |

---

## URC Format Reference

### ^DSCI Format

```
^DSCI: <call_id>,<call_id_type>,<state>,<call_mode>,<mpty>,"<number>",<number_type>
```

| Field | Description |
|-------|-------------|
| call_id | Call identifier (1=MO, 2=MT) |
| call_id_type | 0=MO (origination), 1=MT (termination) |
| state | See ^DSCI state table below |
| call_mode | 0=voice |
| mpty | 0=single party |
| number | Phone number string |
| number_type | 129=national |

### ^DSCI State Codes

| Code | State |
|------|-------|
| 0 | Active |
| 1 | Held |
| 2 | Dialing |
| 3 | Alerting |
| 4 | Incoming |
| 5 | Waiting |
| 6 | Disconnected |

### ^CEND Format

```
^CEND: <call_id>,<call_id_type>,<cause1>,<cause2>,0
```

### ^CEND Cause Codes

| cause1 | cause2 | Scenario |
|--------|--------|----------|
| 101 | 17 | User hangup (ATH/CHUP) |
| 102 | 19 | No answer (timeout) |
| 103 | 17 | Remote busy |
| 105 | 17 | Remote rejected |
| 101 | 3 | Network drop |

---

## Call Flow Simulation

### State Timing

| State | Duration |
|-------|----------|
| DIALING | 3 seconds |
| ALERTING (ringing) | 5 seconds |
| ACTIVE (before voice) | 5 seconds |

### Outgoing Call (MO) - NORMAL Scenario

App sends `ATD10086;` via AT characteristic:

```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATD10086;
0ms       DEV→APP     OK
+3s       DEV→APP     ^ORIG: 1,0
+3s       DEV→APP     ^DSCI: 1,0,2,0,0,"10086",129
+8s       DEV→APP     ^CONF: 1,0
+8s       DEV→APP     ^DSCI: 1,0,3,0,0,"10086",129
+13s      DEV→APP     ^CONN: 1,0
+13s      DEV→APP     ^DSCI: 1,0,0,0,0,"10086",129
```

Voice data begins flowing after ^CONN.

App sends `ATH` or `AT+CHUP` to hang up:
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATH
0ms       DEV→APP     OK
0ms       DEV→APP     ^CEND: 1,0,101,17,0
0ms       DEV→APP     ^DSCI: 1,0,6,0,0,"10086",129
```

### Outgoing Call (MO) - BUSY Scenario

```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATD10086;
0ms       DEV→APP     OK
+3s       DEV→APP     ^ORIG: 1,0
+3s       DEV→APP     ^DSCI: 1,0,2,0,0,"10086",129
+8s       DEV→APP     ^CONF: 1,0
+8s       DEV→APP     ^DSCI: 1,0,3,0,0,"10086",129
+13s      DEV→APP     ^CEND: 1,0,103,17,0
+13s      DEV→APP     ^DSCI: 1,0,6,0,0,"10086",129
```

### Outgoing Call (MO) - NO_ANSWER Scenario

```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATD10086;
0ms       DEV→APP     OK
+3s       DEV→APP     ^ORIG: 1,0
+3s       DEV→APP     ^DSCI: 1,0,2,0,0,"10086",129
+8s       DEV→APP     ^CONF: 1,0
+8s       DEV→APP     ^DSCI: 1,0,3,0,0,"10086",129
~38s      DEV→APP     ^CEND: 1,0,102,19,0
~38s      DEV→APP     ^DSCI: 1,0,6,0,0,"10086",129
```

### Outgoing Call (MO) - REJECT Scenario

```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATD10086;
0ms       DEV→APP     OK
+3s       DEV→APP     ^ORIG: 1,0
+3s       DEV→APP     ^DSCI: 1,0,2,0,0,"10086",129
+8s       DEV→APP     ^CONF: 1,0
+8s       DEV→APP     ^DSCI: 1,0,3,0,0,"10086",129
+11s      DEV→APP     ^CEND: 1,0,105,17,0
+11s      DEV→APP     ^DSCI: 1,0,6,0,0,"10086",129
```

### Outgoing Call (MO) - NETWORK_DROP Scenario

Call connects normally, then drops randomly (5-15s):
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATD10086;
...       ...         (same as NORMAL until CONNECTED at +13s)
+18-28s   DEV→APP     ^CEND: 1,0,101,3,0
+18-28s   DEV→APP     ^DSCI: 1,0,6,0,0,"10086",129
```

### Incoming Call (MT)

Control app triggers `0x78` (SIM_INCOMING):
```
TIME      DIRECTION   DATA
0ms       CTRL→DEV    CMD 0x78 (trigger incoming)
0ms       DEV→CTRL    RESP 0x78 OK
0ms       DEV→APP     RING
0ms       DEV→APP     ^DSCI: 2,1,4,0,0,"10086",129
2000ms    DEV→APP     RING
2000ms    DEV→APP     ^DSCI: 2,1,4,0,0,"10086",129
4000ms    DEV→APP     RING
4000ms    DEV→APP     ^DSCI: 2,1,4,0,0,"10086",129
...       (repeats until ATA or 30s timeout)
30s       DEV→APP     ^CEND: 2,1,102,19,0   (auto timeout)
30s       DEV→APP     ^DSCI: 2,1,6,0,0,"10086",129
```

App sends `ATA` to answer:
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATA
0ms       DEV→APP     OK
0ms       DEV→APP     ^CONN: 2,0
0ms       DEV→APP     ^DSCI: 2,1,0,0,0,"10086",129
```

### Incoming Call (MT) - REJECT Scenario

After app answers with `ATA`, the call is rejected after 3s:
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATA
0ms       DEV→APP     OK
0ms       DEV→APP     ^CONN: 2,0
0ms       DEV→APP     ^DSCI: 2,1,0,0,0,"10086",129
+3s       DEV→APP     ^CEND: 2,1,105,17,0
+3s       DEV→APP     ^DSCI: 2,1,6,0,0,"10086",129
```

### Incoming Call (MT) - NETWORK_DROP Scenario

After app answers, network drops after 5-15s:
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATA
0ms       DEV→APP     OK
0ms       DEV→APP     ^CONN: 2,0
0ms       DEV→APP     ^DSCI: 2,1,0,0,0,"10086",129
5-15s     DEV→APP     ^CEND: 2,1,101,3,0
5-15s     DEV→APP     ^DSCI: 2,1,6,0,0,"10086",129
```

### Hangup During Incoming Call

App sends `ATH` or `AT+CHUP` while RING is active:
```
TIME      DIRECTION   DATA
0ms       APP→DEV     ATH
0ms       DEV→APP     OK
0ms       DEV→APP     ^CEND: 2,1,105,17,0
0ms       DEV→APP     ^DSCI: 2,1,6,0,0,"10086",129
```

---

## AT Commands Intercepted

When simulation is enabled, these GATT-path AT commands are intercepted:

| AT Command | Simulated Response |
|------------|--------------------|
| `ATD<number>;` | Initiates outgoing call flow |
| `ATA` | Answer incoming call |
| `ATH` / `AT+CHUP` | Hang up current call |
| `AT+CSQ` | `+CSQ: <csq>,0` (from SIM_SET_NET) |
| `AT+CREG?` | `+CREG: 0,<creg>` (from SIM_SET_NET) |
| `AT+CPIN?` | `+CPIN: READY, left times = 3` |
| `AT+CIMI` | `+CIMI: 460116072498875` |
| `AT+CCID` | `+CCID: 89860326247551943462` |
| `AT+CGMR` | `+CGMR: HWA BP HS 6.0.5.260206` |
| `AT+CGMM` | `+CGMM: HTDM1310E` |
| `AT+CLCC` | `+CLCC: ...` (call status) |
| `AT+CLIP=...` | `OK` |
| `AT^DSCI=...` | `OK` |
| Other | `OK` |

**Notes**:
- All AT commands have a 100ms response delay to simulate real module timing
- Real module URCs are suppressed while simulation is enabled
- App downlink voice data is dropped until call reaches ACTIVE state

---

## Multi-Connection Support

The simulation supports 2 simultaneous BLE connections:

| Connection Order | Role | Simulation Capability |
|-----------------|------|-----------------------|
| 1st (iOS App) | PRIMARY | Full call control (ATD, ATA, ATH, voice) |
| 2nd (Control App) | DEBUG | SIM_CTRL, SIM_INCOMING, SIM_SET_NET, SIM_GET_STATE |

**Workflow**:
1. iOS app connects first (becomes PRIMARY) - handles call UI
2. Control app connects second (becomes DEBUG) - triggers scenarios
3. Control app sends `0x7A` to check simulation state
4. Control app sends `0x77` to enable simulation + set scenario
5. Control app sends `0x78` to trigger incoming call
6. iOS app receives RING, handles call normally
7. Control app can change scenario mid-session with `0x77`
8. Either client disconnects → advertising restarts for reconnection

---

## Audio Simulation

### Voice (during active call)
- **Format**: PCM 8kHz 16-bit mono, 320 bytes per frame (20ms)
- **Content**: 600Hz sine wave with tremolo + 5% random noise
- **Delivery**: Injected into `voice_data_queue`, processed through normal CODEC2→Base64→BLE pipeline

### Ringtone (during incoming call)
- **Content**: 440Hz + 480Hz dual-tone (North America standard ringback)
- **Pattern**: 1 second on, 2 seconds off
- **Delivery**: Same voice pipeline as above

---

## Typical Test Session

```
# Step 1: iOS app connects (becomes PRIMARY)
# Step 2: Control app connects (becomes DEBUG)

# Step 3: Query simulation state
Control → 0x7A []
# Response: [0x00, 0x00, 0x00] = disabled, NORMAL, IDLE

# Step 4: Enable simulation (NORMAL scenario)
Control → 0x77 [0x01, 0x00]

# Step 5: Verify state
Control → 0x7A []
# Response: [0x01, 0x00, 0x00] = enabled, NORMAL, IDLE

# Step 6: Test outgoing call
iOS app → ATD10086;
# +3s:  ^ORIG + ^DSCI(dialing)
# +8s:  ^CONF + ^DSCI(alerting)
# +13s: ^CONN + ^DSCI(active) → voice
iOS app → ATH
# ^CEND(101,17) → ^DSCI(6)

# Step 7: Change scenario to BUSY
Control → 0x77 [0x01, 0x01]
iOS app → ATD10086;
# +3s: ^ORIG → +8s: ^CONF → +13s: ^CEND(busy)

# Step 8: Test incoming call (NORMAL)
Control → 0x77 [0x01, 0x00]
Control → 0x78 [0x00, 0x00]  # immediate
# iOS app receives RING + ^DSCI(4) every 2s
iOS app → ATA
# ^CONN → ^DSCI(0) → voice

# Step 9: Query state during call
Control → 0x7A []
# Response: [0x01, 0x00, 0x03] = enabled, NORMAL, ACTIVE

# Step 10: Test REJECT scenario on incoming
Control → 0x77 [0x01, 0x05]
Control → 0x78 [0x00, 0x00]
iOS app → ATA
# ^CONN → ^DSCI(0) → (3s) → ^CEND(105,17) → ^DSCI(6)

# Step 11: Test NETWORK_DROP
Control → 0x77 [0x01, 0x03]
Control → 0x78 [0x00, 0x00]
iOS app → ATA
# After 5-15s: ^CEND(101,3) → ^DSCI(6)

# Step 12: Test incoming timeout (no answer)
Control → 0x77 [0x01, 0x00]
Control → 0x78 [0x00, 0x00]
# Wait 30s without ATA
# ^CEND(102,19) → ^DSCI(6)

# Step 13: Disable simulation
Control → 0x77 [0x00, 0x00]
Control → 0x7A []
# Response: [0x00, 0x00, 0x00] = disabled, NORMAL, IDLE
```
