# Bayusuta — build, flash & setup walkthrough

Bayusuta is a two-MCU autopilot for a QuadPlane **tri tiltrotor** (2 front tilting
motors + 1 fixed rear motor):

- **Bayusuta** (this board) — STM32H753 main flight controller running ArduPlane
  (QuadPlane). Onboard: BMI088 + ICM-42688-P IMUs, MS5611 + BMP390 barometers,
  CY15B104Q FRAM (parameters), microSD (logging), 11 PWM outputs, buzzer, ELRS RC
  on UART7, battery monitor on the 6-pin Pixhawk-style power connector (YRRC module).
- **Bayusuta-Periph** — STM32F405 AP_Periph DroneCAN node on the board-internal CAN
  bus. Carries the GPS (USART1), IIS2MDC compass and SHT31 temperature sensor.

Board IDs: `AP_HW_Bayusuta` = 7150, `AP_HW_Bayusuta-Periph` = 7151.

> ⚠️ **Before first flight**: the IMU rotations (`Bayusuta/hwdef.dat`) and compass
> rotation (`Bayusuta-Periph/hwdef.dat`) are `ROTATION_NONE` placeholders — set them
> from the final PCB layout, and calibrate the YRRC power-module scales.

---

## 1. Compile

Environment (once per shell):

```bash
source ~/venv-ardupilot/bin/activate
export PATH=/opt/gcc-arm-none-eabi/bin:$PATH
```

### 1.1 Bootloaders (only needed if not already in `Tools/bootloaders/`)

```bash
Tools/scripts/build_bootloaders.py Bayusuta          # -> Tools/bootloaders/Bayusuta_bl.{bin,elf,hex}
Tools/scripts/build_bootloaders.py Bayusuta-Periph   # -> Tools/bootloaders/Bayusuta-Periph_bl.{bin,elf,hex}
```

### 1.2 Main firmware (ArduPlane / QuadPlane)

```bash
./waf configure --board Bayusuta
./waf plane
```

Outputs in `build/Bayusuta/bin/`:

| File | Use |
|---|---|
| `arduplane.apj` | upload via GCS/uploader through the ArduPilot bootloader |
| `arduplane.bin` | raw app image — flash at **0x0802 0000** (app starts after the 128 KB bootloader sector) |
| `arduplane_with_bl.hex` | **bootloader + firmware combined** — easiest for STM32CubeProgrammer |
| `arduplane.abin` | copy to SD card root as `ardupilot.abin` to flash from the bootloader |

### 1.3 Peripheral firmware (AP_Periph)

```bash
./waf configure --board Bayusuta-Periph
./waf AP_Periph
```

Outputs in `build/Bayusuta-Periph/bin/`:

| File | Use |
|---|---|
| `AP_Periph.apj` | upload via DroneCAN (MissionPlanner DroneCAN screen / `Tools/scripts/CAN/`) |
| `AP_Periph.bin` | raw app image — flash at **0x0801 0000** (app starts after 64 KB) |
| `AP_Periph_with_bl.hex` | **bootloader + firmware combined** — easiest for STM32CubeProgrammer |

---

## 2. Flash with STM32CubeProgrammer (first-time / recovery)

Connection options:

- **ST-LINK / SWD** (recommended): SWD1 header = FCC (H753), SWD2 header = NAVC (F405).
  In CubeProgrammer select *ST-LINK*, mode *Normal*, reset *Hardware reset* → **Connect**.
- **USB DFU** (no ST-LINK needed): hold the BOOT0 switch active (SW5 on the FCC;
  SW3 on the NAVC) while pressing reset, then connect USB (USB-C for FCC, micro-USB
  "USB Debug" for NAVC). In CubeProgrammer select *USB* → **Connect**. Release/restore
  BOOT0 before the final reset.

### 2.1 Main board (H753) — checklist

1. [ ] Connect (SWD or DFU as above).
2. [ ] *(first time only)* **Full chip erase** (Erasing & Programming → Full chip erase).
3. [ ] Open tab *Erasing & Programming* → file = `build/Bayusuta/bin/arduplane_with_bl.hex`.
   The hex contains its own addresses (bootloader @ 0x0800 0000, app @ 0x0802 0000) —
   leave *Start address* blank for hex files.
4. [ ] Tick **Verify programming** → **Start Programming**.
5. [ ] Disconnect, power-cycle. LED A (red, PD6) flashes in the bootloader, then the
   firmware boots; the USB-C port enumerates as an ArduPilot board.

*Two-file alternative (instead of step 3):* program `Tools/bootloaders/Bayusuta_bl.bin`
at `0x08000000`, then `build/Bayusuta/bin/arduplane.bin` at `0x08020000`.

### 2.2 Periph board (F405) — checklist

1. [ ] Connect via SWD2 or DFU (SW3=BOOT0; note SW4 drives BOOT1/PB2 — leave it low).
2. [ ] *(first time only)* Full chip erase.
3. [ ] Program + verify `build/Bayusuta-Periph/bin/AP_Periph_with_bl.hex`
   (or two-file: `Bayusuta-Periph_bl.bin` @ `0x08000000`, `AP_Periph.bin` @ `0x08010000`).
4. [ ] Power-cycle. LED A (PC3) indicates activity.

> Note: a full chip erase on the periph also wipes its parameters (flash sectors 2–3,
> 0x0800 8000–0x0800 FFFF). The embedded `defaults.parm` restores sane defaults
> (GPS on port 3, SHT31 temperature) at first boot.

### 2.3 Everyday updates (no CubeProgrammer)

- **Main**: upload `arduplane.apj` over USB with your GCS, or drop `arduplane.abin`
  on the SD card as `ardupilot.abin`.
- **Periph**: MissionPlanner → Setup → Optional Hardware → DroneCAN → *Update* with
  `AP_Periph.apj` (the FC passes it through over CAN), or SLCAN via the periph's USB.

---

## 3. Tri tiltrotor parameters

All numbers below were taken from the ArduPlane source at the pinned tree
(`quadplane.cpp`, `tiltrotor.cpp`, `AP_MotorsTri.cpp`, `SRV_Channel.h`).

### 3.1 Frame & tilt

| Parameter | Value | Why |
|---|---|---|
| `Q_ENABLE` | `1` | enable QuadPlane (reboot required) |
| `Q_FRAME_CLASS` | `7` | **Tri** — motors are M1 = front-right, M2 = front-left, M4 = rear (from `AP_MotorsTri.cpp`) |
| `Q_TILT_ENABLE` | `1` | enable tiltrotor code (reboot required) |
| `Q_TILT_MASK` | `3` | bit0 (Motor 1) + bit1 (Motor 2) = the two **front** motors tilt |
| `Q_TILT_TYPE` | `2` | *VectoredYaw* — hover yaw comes from differential tilt; the code calls `disable_yaw_torque()` so **no tricopter yaw servo is needed** |
| `Q_TILT_YAW_ANGLE` | `10` (start) | rearward tilt angle at minimum output; must be non-zero for vectored yaw authority. Tune 10–20° |
| `Q_TILT_RATE_UP` | `40` (default) | deg/s tilting back to hover |
| `Q_TILT_RATE_DN` | `0` (default = use RATE_UP) | deg/s tilting to forward flight; set lower (e.g. 15–25) for gentler transitions |
| `Q_TILT_MAX` | `45` (default) | angle at which multicopter control ends during transition |
| `Q_TRANSITION_MS` | `5000` (default) | ms of blended flight after reaching airspeed |
| `Q_TILT_FIX_ANGLE` / `Q_TILT_FIX_GAIN` | `0` / `0` | optional forward-flight thrust vectoring; leave 0 initially |
| `Q_M_PWM_TYPE` | `0` (default) | motor output type; `4`–`7` = DShot150–1200 if you use DShot ESCs (TIM4 group is DShot-capable) |

### 3.2 Output mapping (physical pin → SERVOn → function)

Function numbers from `SRV_Channel.h`.

| hwdef output | MCU pin | Timer | Parameter | Value | Function |
|---|---|---|---|---|---|
| PWM 1 | PD12 | TIM4_CH1 | `SERVO1_FUNCTION` | `33` | Motor 1 — front right |
| PWM 2 | PD13 | TIM4_CH2 | `SERVO2_FUNCTION` | `34` | Motor 2 — front left |
| PWM 3 | PD14 | TIM4_CH3 | `SERVO3_FUNCTION` | `36` | Motor 4 — rear |
| PWM 4 | PD15 | TIM4_CH4 | `SERVO4_FUNCTION` | `0` | spare (future motor slot; inherits the motor group's protocol/rate) |
| PWM 5 | PE9 | TIM1_CH1 | `SERVO5_FUNCTION` | `4` | Aileron left |
| PWM 6 | PE11 | TIM1_CH2 | `SERVO6_FUNCTION` | `4` | Aileron right (set `SERVO6_REVERSED` as installed) |
| PWM 7 | PE13 | TIM1_CH3 | `SERVO7_FUNCTION` | `79` | V-tail left |
| PWM 8 | PE14 | TIM1_CH4 | `SERVO8_FUNCTION` | `80` | V-tail right |
| PWM 9 | PC6 | TIM8_CH1 | `SERVO9_FUNCTION` | `75` | **Tilt motor left** (vectored) |
| PWM 10 | PC7 | TIM8_CH2 | `SERVO10_FUNCTION` | `76` | **Tilt motor right** (vectored) |
| PWM 11 | PA2 | TIM2_CH3 | `SERVO11_FUNCTION` | `56` (= RCIN6, functions 51–66 are RCIN1–16) *or* `0` | drop servo: RC passthrough from a transmitter channel, or `0` + mission/GCS `DO_SET_SERVO` |

Tilt servo setup checklist:

1. [ ] With props OFF, set `SERVO9_MIN/MAX/TRIM` and `SERVO10_MIN/MAX/TRIM` so the
   full servo range maps motors from straight-up (hover) to straight-forward
   (+ `Q_TILT_YAW_ANGLE` of rearward travel). Vectored-yaw tilt outputs are
   0–1000 range outputs (`tiltrotor.cpp` sets `k_tiltMotorLeft/Right` ranges).
2. [ ] Use `SERVOx_REVERSED` so both nacelles move the same physical direction.
3. [ ] Verify in QSTABILIZE (props off): yaw stick right → left nacelle tilts
   forward, right nacelle tilts back.
4. [ ] Verify transition dry-run on the bench with `Q_TILT_RATE_DN` in mind.

### 3.3 Related airframe params

- `ARSPD_TYPE` stays `0` — no airspeed sensor on this build; TECS uses synthetic
  airspeed. Set `Q_ASSIST_SPEED` conservatively once the airframe flies.
- Buzzer (PB0/TIM3) and LEDs need no configuration (`NTF_BUZZ_TYPES` default works).
- RC: ELRS on UART7 = SERIAL5, `SERIAL5_PROTOCOL 23` is already the board default;
  CRSF telemetry works out of the box. `RSSI_TYPE 3` (ReceiverProtocol) is optional.

---

## 4. Sensor fusion parameters

### 4.1 DroneCAN link to Bayusuta-Periph

| Parameter | Value | Note |
|---|---|---|
| `CAN_P1_DRIVER` | `1` | **already the board default** (`HAL_CAN_DRIVER_DEFAULT 1` in hwdef) |
| `CAN_D1_PROTOCOL` | `1` (DroneCAN) | firmware default — verify only |
| `GPS1_TYPE` | `9` | **must set**: GPS = DroneCAN (reboot required) |

Checklist:
1. [ ] Power both MCUs, open MissionPlanner → DroneCAN: the periph node appears with
   a dynamically-allocated node ID (its `CAN_NODE` = 0 → DNA).
2. [ ] Set `GPS1_TYPE 9`, reboot → GPS status becomes *DroneCAN-…* once the periph
   has satellites.
3. [ ] Compass appears automatically (`AP_Compass_DroneCAN`, IIS2MDC, id 0x1E) — run
   onboard compass calibration. `COMPASS_AUTO_ROT 2` (board default) will
   check/correct orientation during calibration.
4. [ ] SHT31 temperature shows as a DroneCAN device temperature (periph publishes at
   `TEMP_MSG_RATE` = 1 Hz).

### 4.2 Dual-IMU EKF3 (both IMUs are on the H753)

These are **firmware defaults** — verify, don't change:

| Parameter | Default | Meaning |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | EKF3 |
| `EK3_ENABLE` | `1` | enabled |
| `EK3_IMU_MASK` | `3` | lane 0 = IMU1 (ICM-42688-P, SPI4), lane 1 = IMU2 (BMI088, SPI1) — one EKF core per IMU with automatic lane switching |
| `INS1_USE` / `INS2_USE` | `1` / `1` | both IMUs feed the EKF |
| `EK3_PRIMARY` | `0` | start on the ICM-42688 lane |

Verification checklist:
1. [ ] Boot messages show both IMUs (`ICM42688`, `BMI088`) and `EKF3 IMU0/IMU1 initialised`.
2. [ ] MissionPlanner EKF status shows two healthy cores; `XKF*` log messages carry `C 0` and `C 1`.
3. [ ] Accel/gyro calibrate with the airframe level (after IMU rotations are finalised!).

### 4.3 Baros, battery, remaining sensors

- Barometers are both local: MS5611 (I2C1, instance 0 = primary) and BMP390 (I2C2,
  backup). `BARO_PRIMARY 0` default; failover is automatic. Verify two baros appear.
- Battery: `BATT_MONITOR 4` is the board default (analog volt+current on PC5/PC4).
  Placeholder scales `BATT_VOLT_MULT 18.182`, `BATT_AMP_PERVLT 36.364` — **calibrate
  against the YRRC power module** with a multimeter/known load.
- No airspeed, no rangefinder, single GPS (`GPS_AUTO_SWITCH` not applicable).
