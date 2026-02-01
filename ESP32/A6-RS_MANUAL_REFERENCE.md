# A6-RS Manual Reference

Extracted from A6-RS_series_servo_drive_manual.pdf for quick code reference.

## Homing (Section 4.1.9)

### Homing Parameters

| Parameter | Name | Range | Default | Unit | Description |
|-----------|------|-------|---------|------|-------------|
| C10.00 | Homing enable | 0-3 | 0 | - | Set homing enable mode |
| C10.01 | Homing method | -2 to 35 | 0 | - | Set homing mode |
| C10.02 | Initial speed | 0-8000 | 100 | rpm | Homing start speed |
| C10.03 | End speed | 0-3000 | 10 | rpm | Homing end speed |
| C10.04 | Accel time | 0-3600000 | 1000 | ms | Homing acceleration |
| C10.06 | Decel time | 0-3600000 | 1000 | ms | Homing deceleration |
| C10.08 | Timeout | 0-(2³²-1) | 60000 | ms | Homing timeout |
| C10.0A | Offset mode | 0-1 | 0 | - | Homing offset mode |
| C10.0B | Offset distance | -2³¹ to 2³¹-1 | 0 | app unit | Homing offset |

### Homing Modes (C10.01 / Register 0x1001)

| Mode | Description |
|------|-------------|
| **-2** | **Forward to mechanical limit, then Z pulse** ← Used by code |
| **-1** | **Reverse to mechanical limit, then Z pulse** ← Used by code |
| 0 | Disabled |
| 1 | Reverse to NL (neg limit), then Z pulse |
| 2 | Forward to PL (pos limit), then Z pulse |
| 3-8 | HSW (home switch) transitions + Z pulse |
| 9-14 | Fixed direction + HSW transitions + Z pulse |
| 15-16 | Reserved |
| 17 | Like 1, but stop at NL (no Z search) |
| 18 | Like 2, but stop at PL (no Z search) |
| 19-30 | Like 3-14, but stop at switch (no Z search) |
| 31-32 | Reserved |
| 33 | Reverse to nearest Z pulse |
| 34 | Forward to nearest Z pulse |
| **35** | **Current position as home** |

### Homing Torque Limit

| Parameter | Register | Description |
|-----------|----------|-------------|
| C10.30 | 0x1030 | Homing torque limit (0.1%/LSB) |

## Position Control Parameters

### Electronic Gear Ratio (Section 4.1.3)

| Parameter | Register | Description |
|-----------|----------|-------------|
| C03.04 | 0x0304 | Gear ratio denominator (steps per rev) |
| C03.06 | 0x0306 | Gear ratio numerator (encoder counts/rev = 131072) |

### Position Reference Filter

| Parameter | Register | Description |
|-----------|----------|-------------|
| C01.22 | 0x0122 | Position input LPF (0.1ms/LSB, code uses 50 = 5.0ms) |

### Pulse Channel Selection

| Parameter | Register | Description |
|-----------|----------|-------------|
| C00.22 | 0x0022 | Pulse channel (1 = high-speed) — requires power cycle |

## Servo Control

### Enable/Disable

| Parameter | Register | Value | Description |
|-----------|----------|-------|-------------|
| C04.11 | 0x0411 | 0 | Disable servo |
| C04.11 | 0x0411 | 1 | Enable servo |

### Position Limits

| Parameter | Register | Description |
|-----------|----------|-------------|
| C06.07 | 0x0607 | Limit active mode (2 = after homing) |
| C06.08 | 0x0608 | Positive position limit (32-bit counts) |
| C06.0A | 0x060A | Negative position limit (32-bit counts) |

### Position Deviation

| Parameter | Register | Description |
|-----------|----------|-------------|
| C06.00 | 0x0600 | Excessive position deviation threshold |

## Torque Control

### Torque Limits

| Parameter | Register | Description |
|-----------|----------|-------------|
| C03.43 | 0x0343 | Negative torque limit (0.1%/LSB) |
| C03.44 | 0x0344 | Positive torque limit (0.1%/LSB) |

## Monitoring (Read-Only)

| Parameter | Register | Description |
|-----------|----------|-------------|
| U40.01 | 0x4001 | Actual speed (RPM, int16) |
| U40.16 | 0x4016 | Actual position (counts, int32) |

## Communication Parameters

### RS485 Settings

| Parameter | Register | Value | Description |
|-----------|----------|-------|-------------|
| C0A.05 | 0x0A05 | 0 | Don't save RS485 writes to EEPROM |
| C0A.06 | 0x0A06 | 1 | Word order: high word first (required for 32-bit) |

## Modbus Protocol

### Frame Format

- **Baud**: 115200 (typical)
- **Format**: 8N1
- **Slave ID**: 1-247

### Command Codes

| Code | Operation |
|------|-----------|
| 0x03 | Read 16-bit or 32-bit parameters |
| 0x06 | Write 16-bit parameters only |
| 0x10 | Write 32-bit parameters only |

### Address Format

Register address = `(Group << 8) | Offset`

Example: C06.11 → Group=0x06, Offset=0x11 → Address=0x0611

### 32-bit Parameter Handling

- Read: Request 2 words starting at parameter address
- Write: Use command 0x10 with 2 words (high word first when C0A.06=1)

## Commissioning (Section 5.3)

1. **Power on** the servo drive
2. **Jogging** — Test motor rotation
3. **Set parameters** — Configure control mode, gains
4. **Servo running** — Enable and verify operation
5. **Stop** — Disable when done

## Inertia Auto-Tuning (Section 6.2)

Parameters in group C07:
- C07.00: Auto-tuning mode
- C07.01: Auto-tuning speed
- C07.03: Auto-tuning target torque
- C07.04: Number of auto-tuning turns

## Gain Parameters

| Parameter | Register | Description |
|-----------|----------|-------------|
| C00.05 | 0x0005 | Stiffness level (recommended: 20) |
| C00.02 | 0x0002 | Simple gear ratio (set to 0 to disable) |

## Fault and Alarm Codes (Chapter 7)

### Fault Categories

| Category | Fault Codes | Resettable |
|----------|-------------|------------|
| Class 1 | Er0x.x to Er2x.x | Non-resettable (most) |
| Class 2 | Er4x.x to Er5x.x | Resettable |
| Class 3 | Er8x.x to ErAx.x, ALFxx | Resettable |

### Reset Methods

- **Software reset**: Set F31.00 = 1
- **Clear fault history**: Set F31.04 = 1
- **Encoder reset**: Set F31.10 = 3 (encoder fault) or F31.10 = 4 (multi-turn reset)

### Class 1 Faults (Non-resettable - Hardware/Critical)

| Code | Name | Cause | Solution |
|------|------|-------|----------|
| Er01.0 | Software version mismatch | MCU/FPGA versions incorrect | Update firmware |
| Er01.1 | Motor parameter mismatch | Incorrect motor parameters | Use matching motor |
| Er02.0 | No specified drive | Drive model incorrect | Contact support |
| Er02.1 | No specified motor | Motor model incorrect | Contact support |
| Er02.5 | Drive 5V too low | Internal voltage low | Replace drive |
| Er03.0 | System parameter error | Software updated | Restore defaults (F31.02=1) |
| Er03.1 | Parameter out-of-range | Address error | Check parameter access |
| Er03.2 | Parameter writing error | Frequent writes, power unstable | Check communication/power |
| Er03.3 | Parameter reading error | Frequent reads, drive faulty | Check communication |
| Er04.0 | FPGA power-on check error | FPGA failure | Replace drive |
| Er04.1 | FPGA power-on not reset | FPGA failure | Replace drive |
| Er04.2 | FPGA transmission interrupted | FPGA/MCU timeout | Replace drive |
| Er05.0 | Current loop timeout | MCU scheduling abnormal | Replace drive |
| Er05.1 | Speed loop timeout | MCU scheduling abnormal | Replace drive |
| Er05.2 | Position loop timeout | MCU scheduling abnormal | Replace drive |
| Er06.0 | Runaway protection | UVW wiring wrong, encoder error | Check phase sequence, encoder |
| Er10.0 | P-hardware overcurrent | Gain too high, motor oscillation | Adjust gain, check cables |
| Er10.1 | N-hardware overcurrent | Gain too high, braking resistor | Adjust gain, check resistor |
| Er10.2 | U phase overcurrent | Motor cables shorted/grounded | Check/replace motor cables |
| Er10.3 | V phase overcurrent | Motor cables shorted/grounded | Check/replace motor cables |
| Er10.4 | Output short to ground | UVW shorted to ground | Check cables, replace drive |
| Er10.5 | Current sampling failure | Interference, chip damaged | Check grounding, add ferrites |
| Er10.6 | Current parameter error | Incorrect sampling params | Set R21.24=0 |
| Er10.7 | UV current correction failure | Correction accuracy >5% | Replace drive |
| Er10.8 | Current zero drift | Zero drift too high | Replace drive |
| Er10.9 | Current exception on enable | Sampled current too large | Connect motor cable |
| Er11.0 | Motor speed too high on power-on | Motor rotating at power-on | Keep motor stationary |
| Er11.1 | Drive over-temperature | Ambient temp too high | Improve cooling |
| Er12.0 | PWM buffer detection failure | Detection failed at power-on | Contact support |
| Er20.1 | Encoder internal fault | Encoder failure | Reset (F31.10=3), replace motor |
| Er20.2 | Encoder read/write error | Data exchange error | Replace encoder cable |
| Er20.3 | Encoder data frame loss | Cable abnormal, interference | Replace cable, add ferrites |
| Er20.4 | Encoder incremental position error | Single-turn position abnormal | Separate motor/encoder cables |
| Er20.5 | Abnormal encoder data | Internal params abnormal | Separate cables, replace motor |
| Er20.6 | Encoder type mismatch | Motor model mismatch | Use matching motor |
| Er20.7 | Encoder model not supported | Unsupported encoder | Use matching motor |
| Er20.8 | Encoder battery failure | Battery voltage low | Replace battery, reset (F31.10=4) |
| Er20.9 | Encoder multi-turn error | Multi-turn counting error | Reset (F31.10=4), replace motor |
| Er21.0 | Encoder pulses mismatch | Pulses/rev mismatch | Redistribute encoder params |
| Er22.0 | Second encoder disconnection | Disconnected | Check second encoder cable |
| Er22.1 | BISS encoder timeout | Baud rate too low | Increase C1B.1E |
| Er22.2 | BISS communication error | Data error | Use shielded twisted pair |
| Er22.3 | BISS check error | Check error | Use shielded twisted pair |
| Er22.4 | BISS internal error | Encoder failure | Replace BISS encoder |
| Er22.5 | BISS power-on read error | Position read error | Use shielded twisted pair |

### Class 2 Faults (Resettable)

| Code | Name | Cause | Solution |
|------|------|-------|----------|
| Er40.0 | Drive overload | Load rate too high | Reduce load or use higher power drive |
| Er41.0 | Motor overload | Continuous high torque | Reduce load, increase accel time |
| Er41.1 | Motor over-temp (locked rotor) | UVW phase loss, locked rotor | Check cables, eliminate jamming |
| Er41.2 | Motor over-temp (PTC) | PTC detected high temp | Check PTC wiring, disable if unused |
| Er41.3 | Motor winding temp high | Winding too hot | Improve cooling, reduce load |
| Er42.0 | IGBT temp too high | Ambient temp high | Improve cooling, wait before reset |
| Er42.1 | Discharge tube temp high | Too many discharge cycles | Control discharge frequency |
| Er42.2 | Heatsink temp too high | Ambient temp high, fan fault | Check fan, improve cooling |
| Er43.0 | Overvoltage | Input voltage high, braking resistor | Check supply, braking resistor |
| Er43.1 | Undervoltage | Power unstable, phase loss | Check power supply, wiring |
| Er45.0 | S-ON enable failure | Multiple sources enabling | Use single enable source |
| Er46.0 | Motor overspeed | UVW wrong, threshold too low | Check phase sequence, C06.03 |
| Er47.0 | Excessive position deviation | Locked rotor, gain low | Eliminate jamming, adjust gain |
| Er47.1 | Position deviation overflow | Same as Er47.0 | Same as Er47.0 |
| Ex47.2 | Fully closed-loop mixed deviation | C1B.08 too small | Increase C1B.08 |
| Er48.0 | STO status exception | STO fault | Check STO wiring |
| Er48.1 | STO buffer 5V fault | Buffer power abnormal | Replace drive |
| Er48.2 | STO optocoupler fault | Upstream optocoupler short | Check STO wiring |
| Er48.3 | STO buffer detection failure | Buffer switch fault | Replace drive |
| Er49.0 | Output phase loss | UVW disconnection | Replace motor cable |
| Er50.1 | D/Q current overflow | Sampling error | Replace drive |
| Er51.0 | Inertia auto-tuning failure | Vibration, loose connection | Enable vibration suppression |
| Er51.1 | Inertia parameter error | Torque too large | Reduce C07.01, C07.03 |
| Er52.0 | Angle auto-tuning failure | Tuning failed | Check motor params, rewire |
| Er53.0 | Motor param auto-tuning timeout | Timeout | Contact support |
| Er53.1 | Resistance auto-tuning failure | Failed | Contact support |
| Er53.2 | Inductance auto-tuning failure | Failed | Contact support |
| Er53.3 | Back EMF auto-tuning failure | Failed | Contact support |
| Er54.0 | Current loop auto-tuning failure | Failed | Contact support |
| Er55.0 | Excessive vibration | Vibration too high | Adjust gain parameters |
| Er57.0 | Friction auto-tuning failed | C07.28-C07.2F unreasonable | Adjust friction params |

### Class 2 Faults (Configuration/Software)

| Code | Name | Cause | Solution |
|------|------|-------|----------|
| Er80.0 | Control power undervoltage | Power unstable | Check control power |
| Er81.0 | Input phase loss 1 | Phase loss | Check three-phase supply |
| Er81.1 | Input phase loss 2 | Phase loss | Check three-phase supply |
| Er82.0 | DI function allocation fault | Same function on multiple DIs | Use unique function numbers |
| Er82.1 | DO function allocation fault | Function number invalid | Restore defaults (F31.02=1) |
| Er82.2 | VDI function allocation fault | Same function on DI and VDI | Use unique allocations |
| Er82.3 | VDO function allocation fault | Same function on DO and VDO | Use unique allocations |
| Er82.4 | Position capture DI error | DI7 not set for capture | Set C04.18=33 or C10.20=0 |
| Er83.0 | AI1 sampling overvoltage | Input >10.23V | Reduce input, use shielded cable |
| Er83.1 | AI2 sampling overvoltage | Input >10.23V | Reduce input, use shielded cable |
| Er83.2 | AI1 chip fault | Chip missing/clock fail | Replace drive |
| Er83.3 | AI2 chip fault | Chip missing/clock fail | Replace drive |
| Er83.4 | Current-type AI disconnected | AI2 current too low | Check wiring |
| Er83.5 | Current-type sampling overcurrent | AI2 current too large | Reduce current |
| Er84.0 | Electronic gear ratio error | Ratio exceeds limit | Set within (0.001, 4000×res/10000) |
| Er84.1 | Software limit error | Lower ≥ upper limit | Set min < max |
| Er84.2 | Encoder resolution error | Resolution abnormal | Restore defaults (F31.02=1) |
| Er84.3 | Home position error | Offset beyond limits | Set offset within limits |
| Er85.1 | Freq division pulse error | Output freq >4MHz | Reduce C00.28 |
| Er87.0 | Position reference increment error | Increment too large | Reduce reference increment |
| Er87.3 | Target position overflow | 32-bit overflow during limiting | Reduce target position |
| Er87.4 | Target exceeds single-turn max | Position >231 in rotation mode | Adjust gear ratio |
| Er87.5 | Fully closed-loop param error | C00.07=4/5 with C1B.00≠0 | Don't use absolute rotation |
| ErA0.1 | Multi-turn overflow | Turns >32767 or <-32768 | Reset (F31.10=4), re-home |

### Alarms (Class 3 - All Resettable)

| Code | Name | Cause | Solution |
|------|------|-------|----------|
| ALF0.0 | Emergency stop | E-stop DI active | Clear E-stop signal |
| ALF1.0 | Re-power-on required | Parameter needs restart | Power cycle |
| ALF1.1 | Frequent parameter storage | Too many EEPROM writes | Reduce write frequency |
| ALF1.2 | Torque reached param error | DO threshold wrong | Set C03.4A > C03.4B |
| ALF2.0 | Forward overtravel | PL limit active | Move reverse, check limits |
| ALF2.1 | Reverse overtravel | NL limit active | Move forward, check limits |
| ALF3.0 | AI1 zero offset too large | Wiring error, interference | Use shielded cable |
| ALF3.1 | AI2 zero offset too large | Wiring error, interference | Use shielded cable |
| ALF4.0 | Homing timeout | Time exceeded | Adjust speed/timeout, check signals |
| ALF4.1 | Homing DI conflict | Both limits or home+limit active | Check signal wiring |
| ALF4.2 | Homing mode conflict | C10.01 set incorrectly | Check homing mode |
| ALF5.0 | Braking resistor overload | Cable loose, resistance too high | Check connections, resistance |
| ALF5.1 | Braking resistance too small | Resistance < minimum | Use resistor ≥ minimum |
| ALF6.0 | Freq division output error | Setting error | Check output settings |
| ALF6.1 | Output phase loss | Current abnormal | Check power cable |
| ALF7.0 | Position compare segment warning | Start > end segment | Check comparison settings |
| ALF7.3 | Position compare out of range | Initial segment out of range | Check cyclic absolute settings |
| ALF8.0 | Vibration during auto-tuning | Vibration detected | Enable vibration suppression |
| ALF9.0 | Encoder battery low | Battery voltage low | Replace battery |
| ALFA.0 | Drive high temp warning | Temperature elevated | Improve cooling |
| ALFB.0 | Brake PMOS short | PMOS failure | Check brake circuit |
| ALFB.1 | Brake NMOS short | NMOS failure | Check brake circuit |
| ALFB.2 | Brake 24V disconnection | 24V open circuit | Check brake wiring |
| xxnr | Servo not ready | Not ready | Check enable conditions |

### Bus Fault Codes (EtherCAT/CANopen)

| Code | Name |
|------|------|
| 0x0000 | No fault |
| 0x2312 | Continuous current fault |
| 0x2330 | Short circuit to ground |
| 0x3120 | Control power overvoltage |
| 0x3130 | Phase loss |
| 0x3210 | Main circuit overvoltage |
| 0x3220 | Main circuit undervoltage |
| 0x3230 | Overload |
| 0x4210 | Over-temperature |
| 0x5443 | Forward overtravel |
| 0x5444 | Reverse overtravel |
| 0x5530 | Storage fault |
| 0x6100 | Internal error |
| 0x6310 | Current overflow |
| 0x6320 | Parameter error |
| 0x7121 | Motor locked-rotor |
| 0x7122 | Motor mismatch |
| 0x7305 | Encoder error |
| 0x7500 | Communication fault |
| 0x7600 | Data storage |
| 0x8400 | Speed control |
| 0x8611 | Following fault (position deviation) |
| 0x8220 | Length error |
| 0x8700 | Sync controller |
| 0x8900 | Process data monitoring |
| 0x0FFF | Factory fault |

### Monitoring Registers for Diagnostics

| Parameter | Register | Description |
|-----------|----------|-------------|
| U41.00 | 0x4100 | Current fault code |
| U41.01 | 0x4101 | Fault history 1 (most recent) |
| U41.02 | 0x4102 | Fault history 2 |
| U41.03 | 0x4103 | Fault history 3 |
| U41.04 | 0x4104 | Fault history 4 |
| U41.05 | 0x4105 | Fault history 5 |
| U41.06 | 0x4106 | Error group number |
| U41.07 | 0x4107 | Error offset |

---

*Source: A6-RS_series_servo_drive_manual.pdf*
