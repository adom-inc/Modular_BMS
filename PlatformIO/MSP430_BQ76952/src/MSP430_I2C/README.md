# BQ76952EVM + MSP430F5529 BMS Firmware

Firmware for the **BQ76952EVM** using an **MSP430F5529** as the I²C host. This project is based on TI’s BQ769x2 demo framework and adds higher-level helpers for:

- n-cell configuration using `VCellMode`
- periodic measurement polling and alarm handling
- CHG/DSG FET control from two GPIO inputs (P3.3 and P3.4)
- structured output in **human-readable** or **JSON** notification format in Hydrogen Style

---

## Hardware

- **MSP430F5529LP** eval board (for programming over SBW)
- **MSP430F5529** (host MCU)
- **BQ76952EVM** (battery monitor + protection + balancing)
- I²C: UCB1 mapped to **P4.4=SCL**, **P4.5=SDA**
- UART: UCA1 mapped to **P4.2=RX**, **P4.3=TX**
- FET control inputs:
  - **P3.3** → enable **Discharge** (DSG)
  - **P3.4** → enable **Charge** (CHG)
  - Both are configured as GPIO inputs with pulldown resistors

---

## Build / Run

This code is intended to be built in PIO toolchain using the MSP430F5529 eval board for SBW programming.

At startup, `main()`:
1. Stops the watchdog
2. Initializes GPIO, port mapping, UART, I²C
3. Resets the BQ76952
4. Applies RAM configuration via `BQ769x2_Init()`
5. Enables FET control and disables sleep for full-speed measurement updates
6. Enters the main polling loop

---

## Configuration

### Cell Count

This project is configured for **3 cells** using `VCellMode`:

```c
#define NUM_CELLS 3
#define NUM_CELLS_BITS 0x0007   // enable Cell1..Cell3
```

`NUM_CELLS_BITS` is written to the `VCellMode` register inside `BQ769x2_Init()`.

---

## Output Modes

A global `output_mode` controls whether telemetry is printed as:
- `OUTPUT_HUMAN`: readable console logs via `printf`
- `OUTPUT_JSON`: JSON notification packets (Hydrogen-style wrapper)

Select mode:

```c
set_output_mode(OUTPUT_HUMAN);
// or
set_output_mode(OUTPUT_JSON);
```

### JSON Wrapper

JSON output is wrapped in a common “notification” envelope via:

- `send_json_notification_data(method, id, data_str)`
- `send_json_notification_fmt(method, id, data_fmt, ...)`

All BMS-specific output functions ultimately route through these.

---

## Key High-Level BQ76952 Functions

### `BQ769x2_Init()`

Applies the core BQ76952 RAM configuration:
- enters `CONFIG_UPDATE`
- sets power / regulator / TS pin config
- sets `VCellMode` for your pack (3S by default)
- enables protections (SCD/OCD/OV/UV/etc.)
- sets alarm mask, balancing config, thresholds
- exits `CONFIG_UPDATE`

**Used in `main()` once after reset**, before enabling FETs.

---

### `BQ769x2_ReadAlarmStatus()`

Reads `AlarmStatus` (direct command) and returns a 16-bit bitmask.

In `main()` this is used as the “scheduler”:
- if FULLSCAN bit is set → new measurements are ready
- if SafetyStatus alarm bits set → fetch safety status

---

### `BQ769x2_ReadAllVoltages()`

Updates:
- `CellVoltage[0..15]` from `Cell1Voltage..Cell16Voltage`
- `Stack_Voltage` from `StackVoltage`
- `Pack_Voltage` from `PACKPinVoltage`
- `LD_Voltage` from `LDPinVoltage`

**Important note:** on the EVM, `Pack_Voltage` is the PACK pin *after* the FETs, so it may read ~0V when DSG/CHG FETs are open.
**Important note:** on the EVM, `Stack_Voltage` is what we would usually call the "Battery Pack" voltage. Basically the voltage from top cell to GND. 

---

### `BQ769x2_ReadVoltage(command)`

Generic “direct command” voltage read:
- Cell voltages are returned in **mV**
- Stack/Pack/LD voltages are treated as **0.01V units** and converted to mV

---

### `BQ769x2_ReadCurrent()`

Reads `CC2Current` (direct command) and returns the raw 16-bit value.

**Current direction:** CC2 current is signed (two’s complement).

Recommended usage in this codebase:

```c
int16_t current_mA = (int16_t)BQ769x2_ReadCurrent();
// positive = charging, negative = discharging
```

---

### `BQ769x2_ReadTemperature(command)`

Reads a temperature direct command (e.g., `TS1Temperature`) and converts to °C:

```c
Temperature_C = 0.1 * raw_K - 273.15
```

---

## Protection / Fault Monitoring

### `BQ769x2_ReadSafetyStatus()`

Reads:
- `SafetyStatusA`
- `SafetyStatusB`
- `SafetyStatusC`

And derives high-level flags:
- `UV_Fault`, `OV_Fault`, `SCD_Fault`, `OCD_Fault`
- `ProtectionsTriggered`

In `main()`, safety status is checked whenever AlarmStatus indicates safety-related bits or when a protection was previously active.

---

### `BQ769x2_ReadPFStatus()`

Reads Permanent Failure status registers A/B/C. Useful when diagnosing “fuse LED” or persistent shutdown behavior.

---

## FET Control (CHG/DSG)

### `BQ769x2_ReadFETStatus()`

Reads `FETStatus` and updates globals:
- `CHG`, `DSG`, `PCHG`, `PDSG`

Use this to confirm the actual driver states.

---

### `BQ769x2_WriteFETStatus(bool DSG, bool CHG)`

Controls CHG/DSG using the **FET_CONTROL** subcommand.

This function uses the BQ76952 convention:
- writing `DSG_OFF=1` forces DSG OFF
- writing `CHG_OFF=1` forces CHG OFF

So the function takes **enable booleans** and internally writes the “OFF” bits as needed.

Example:

```c
BQ769x2_WriteFETStatus(true, false);  // DSG enabled, CHG forced off
```

> Note: this is host control. The BQ76952 may still keep FETs open if protections are active (UV/OV/OCD/SCD/PF).

---

### `update_fets_from_buttons()`

Reads the two GPIO inputs and applies FET control:
- `P3.3 high` → enable discharge (DSG)
- `P3.4 high` → enable charge (CHG)

Then:

```c
BQ769x2_WriteFETStatus(dsg_enable, chg_enable);
```

This is called periodically from `main()` (currently every 20 cycles).

---

## Output Helpers (Human + JSON)

These functions produce either console output or JSON notifications depending on `output_mode`.

### `output_bms_config(battery_type, cell_count)`

Emits a config packet at startup and periodically.

### `output_bms_voltages(pack_mV, CellVoltage[], cell_count)`

Emits pack + per-cell voltages.

### `output_fet_status(chg, dsg)`

Emits CHG/DSG states as booleans/ints.

### `output_current_value(current_mA)`

Emits signed current in mA.

---

## Main Loop Behavior (Current Program)

The `while(1)` loop does:

1. Read `AlarmStatus`
2. If FULLSCAN complete → read voltages/current/temps → clear FULLSCAN bit
3. If safety alarm bits present → read safety status → clear alarm bits
4. Every ~20 cycles:
   - output voltages
   - update FETs from button inputs
   - read and output FET states
   - read and output current
5. Every ~100 cycles:
   - re-emit config and print faults if present
6. Delay ~20 ms

---

## How to Write Your Own `main()`

A minimal, clean template:

```c
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    GPIO_initPins();
    map_I2C_and_UART();
    UART_Init();
    I2C_initModule();

    set_output_mode(OUTPUT_JSON);

    CommandSubcommands(BQ769x2_RESET);
    delayUS(60000);

    BQ769x2_Init();
    delayUS(10000);

    CommandSubcommands(FET_ENABLE);
    CommandSubcommands(SLEEP_DISABLE);

    while (1)
    {
        uint16_t alarms = BQ769x2_ReadAlarmStatus();

        if (alarms & 0x80) {
            BQ769x2_ReadAllVoltages();
            int16_t current_mA = (int16_t)BQ769x2_ReadCurrent();

            // clear FULLSCAN complete bit
            DirectCommands(AlarmStatus, 0x0080, W);

            output_bms_voltages(Stack_Voltage, CellVoltage, NUM_CELLS);
            output_current_value(current_mA);
        }

        update_fets_from_buttons();

        if (alarms & 0xC000) {
            BQ769x2_ReadSafetyStatus();
            // clear safety alarm bits
            DirectCommands(AlarmStatus, 0xF800, W);
        }

        delayUS(20000);
    }
}
```

Recommended approach:
- Drive logic from `AlarmStatus` FULLSCAN rather than reading constantly.
- Keep FET control separate (e.g., button read each loop).
- Treat CC2 current as **signed** (cast to `int16_t`).

---

## Common Gotchas

- **PACK pin voltage reading 0V** is normal if the PACK node is not being driven and/or FETs are open.
- **Current is signed**; treat `CC2Current` as two’s-complement.
- Protections (UV/OV/OCD/SCD/PF) can override host FET commands.
- If the EVM “FUSE” LED is on, check PF status and PF-related configuration.

---

## Module Notes

- `BQ769x2Header.h`: contains direct command IDs, subcommands, and RAM register addresses
- Low-level comms:
  - `I2C_ReadReg()`, `I2C_WriteReg()`, ISR state machine
  - `DirectCommands()`, `Subcommands()`, `CommandSubcommands()`
