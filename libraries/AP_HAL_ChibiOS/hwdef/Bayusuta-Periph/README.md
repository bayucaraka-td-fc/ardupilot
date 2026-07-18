# Bayusuta-Periph

STM32F405 AP_Periph DroneCAN node paired with the Bayusuta H753 flight controller
over the board-internal CAN bus. It provides:

- GPS on USART1 (PB6/PB7), `GPS_PORT 3` via embedded defaults
- IIS2MDC compass on I2C2 @0x1E (rotation TODO — set from PCB layout)
- SHT31 temperature on I2C2 @0x44, published as DroneCAN device temperature
- USB (SLCAN/debug), CH330N debug console on USART3, spare UART4 + external I2C3

Board ID `AP_HW_Bayusuta-Periph` (7151). Node ID via dynamic allocation (`CAN_NODE 0`).

Build and flash instructions, including STM32CubeProgrammer steps and the full
parameter walkthrough, live in the main board's README:
`libraries/AP_HAL_ChibiOS/hwdef/Bayusuta/README.md`.

Quick build:

```bash
./waf configure --board Bayusuta-Periph
./waf AP_Periph
# combined image for CubeProgrammer: build/Bayusuta-Periph/bin/AP_Periph_with_bl.hex
```
