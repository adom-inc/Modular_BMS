#include "msp430f5529.h"
#define ACTIVE
#ifdef ACTIVE

/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//******************************************************************************
//   BQ76952EVM demo code for MSP430FR2x55 + BQ769x2
//
//   Description: MSP430FR2x55 functions as the I2C host that communicates
//   with the BQ769x2 sending and receiving different types of commands.
//   ACLK = 32.768kHz REFO or external XT1, MCLK = SMCLK = 1MHz DCO
//
//                                     /|\ /|\
//                    MSP430FR2x55     10k  |
//                 -----------------    |  10k
//            /|\ |             P4.5|---+---|-- I2C Data (UCB1SDA)
//             |  |                 |       |
//             ---|RST          P4.4|-------+-- I2C Clock (UCB1SCL)
//                |                 |
//                |             P2.3|---> SHUT
//                |                 |
//                |             P4.2|---> UART RX
//                |                 |
//       LEDs <---|P8.0 -8.2       P4.3|<--- UART TX
//                |                 |
//
//
//   Andrew Han and Matt Sunna
//   Texas Instruments Inc.
//   August 2021
//   Built with Code Composer Studio (CCS) v10.1.1
//******************************************************************************

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <assert.h>
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "BQ769x2Header.h"

//------------------------------------------------------------------------------
// Defines / macros
//------------------------------------------------------------------------------

// USER CONFIG HERE
#define NUM_CELLS 3
#define NUM_CELLS_BITS 0x0007 // bit 0 for cell 0, bit 3 for cell 3, etc. Write 1 to enable, write 0 to disable


// other 
#define DEV_ADDR 0x08 // BQ769x2 7-bit I2C address
#define CRC_Mode 0    // 0 = disabled, 1 = enabled

#define MAX_BUFFER_SIZE 10
#define R 0  // Read
#define W 1  // Write
#define W2 2 // Write data with two bytes

// Porting macros for F5529: match FR-style USCI flags
#define USCI_I2C_UCRXIFG0 USCI_I2C_UCRXIFG
#define USCI_I2C_UCTXIFG0 USCI_I2C_UCTXIFG

// These extra FIFO levels don't exist on F5529; they will never be hit
#define USCI_I2C_UCRXIFG1 0x12
#define USCI_I2C_UCRXIFG2 0x14
#define USCI_I2C_UCRXIFG3 0x16
#define USCI_I2C_UCTXIFG1 0x18
#define USCI_I2C_UCTXIFG2 0x1A
#define USCI_I2C_UCTXIFG3 0x1C

// Upper bound for __even_in_range
#define USCI_I2C_UCBIT9IFG USCI_I2C_UCTXIFG

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef enum I2C_ModeEnum
{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

typedef enum
{
    OUTPUT_HUMAN = 0,
    OUTPUT_JSON = 1
} OutputMode;

//------------------------------------------------------------------------------
// Globals
//------------------------------------------------------------------------------

// UART output mode

static OutputMode output_mode = OUTPUT_HUMAN; // default to human-readable output

// I2C state machine
I2C_Mode MasterMode = IDLE_MODE;
uint8_t TransmitRegAddr = 0;
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

// RX / measurement buffers
uint8_t RX_data[2] = {0x00, 0x00};
uint8_t RX_32Byte[32] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint16_t CellVoltage[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

float Temperature[3] = {0, 0, 0};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
uint16_t Pack_Current = 0x00;
uint16_t AlarmBits = 0x00;

uint8_t value_SafetyStatusA;
uint8_t value_SafetyStatusB;
uint8_t value_SafetyStatusC;
uint8_t value_PFStatusA;
uint8_t value_PFStatusB;
uint8_t value_PFStatusC;
uint8_t FET_Status;
uint16_t CB_ActiveCells;

uint8_t UV_Fault = 0;
uint8_t OV_Fault = 0;
uint8_t SCD_Fault = 0;
uint8_t OCD_Fault = 0;
uint8_t ProtectionsTriggered = 0;

uint8_t LD_ON = 0;
uint8_t DSG = 0;
uint8_t CHG = 0;
uint8_t PCHG = 0;
uint8_t PDSG = 0;

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------

// Board / low-level
void GPIO_initPins(void);
void GPIO_configPins(void);
void LED_On(int32_t ledPin);
void LED_Off(int32_t ledPin);
void UART_Init(void);
void I2C_initModule(void);
void map_I2C_and_UART(void);

// Utility
void delayUS(uint16_t us);
int putchar(int c);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
unsigned char Checksum(unsigned char *ptr, unsigned char len);

// I2C access
I2C_Mode I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
I2C_Mode I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

// BQ769x2 API
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);
void CommandSubcommands(uint16_t command);
void Subcommands(uint16_t command, uint16_t data, uint8_t type);
void DirectCommands(uint8_t command, uint16_t data, uint8_t type);
void BQ769x2_Init(void);

void BQ769x2_ReadFETStatus(void);
void BQ769x2_WriteFETStatus(bool DSG, bool CHG);
void update_fets_from_buttons(void);
void BQ769x2_ShutdownPin(void);
void BQ769x2_ReleaseShutdownPin(void);

uint16_t BQ769x2_ReadAlarmStatus(void);
void BQ769x2_ReadSafetyStatus(void);
void BQ769x2_ReadPFStatus(void);

uint16_t BQ769x2_ReadVoltage(uint8_t command);
void BQ769x2_ReadAllVoltages(void);
uint16_t BQ769x2_ReadCurrent(void);
float BQ769x2_ReadTemperature(uint8_t command);

// JSON/Hydrogen packets

void send_json_notification(const char *method,
                            const char *id,
                            const char *data_fmt, ...);

// --- 1. BMS output messages ---------------------------------------------------

void output_bms_config(const char *battery_type, int cell_count);
void output_bms_voltages(uint32_t v_mV, const uint16_t *CellVoltage, int cell_count);
void output_fet_status(bool chg, bool dsg);
void output_current_value(int16_t current_mA);

// --- output mode selection --------------------------------------------------

void set_output_mode(OutputMode mode)
{
    output_mode = mode; // set global output mode
}

//------------------------------------------------------------------------------
// main
//------------------------------------------------------------------------------
int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    GPIO_initPins();
    map_I2C_and_UART();
    UART_Init();
    I2C_initModule();

    LED_On(0); // Popwer LED

    set_output_mode(OUTPUT_JSON);  // or OUTPUT_JSON

    CommandSubcommands(BQ769x2_RESET); // Reset the BQ769x2 registers
    delayUS(60000);

    BQ769x2_Init(); // Configure all BQ769x2 RAM settings
    delayUS(10000);

    BQ769x2_WriteFETStatus(0,1);
    CommandSubcommands(FET_ENABLE); // Enable CHG and DSG FETs
    delayUS(10000);

    CommandSubcommands(SLEEP_DISABLE); // Disable Sleep for full-speed measurements

    // Wait for FETs to close and measurements to settle
    delayUS(60000);
    delayUS(60000);
    delayUS(60000);
    delayUS(60000);

    delayUS(20000);
    delayUS(20000);
    delayUS(20000);

    LED_On(1);
    printf("hello world\n\r");
    output_bms_config("Li-ion", 3);


    while (1)
    {
        static int i = 0; // Loop counter for periodic output 

        // Read alarm status and check for new measurements
        AlarmBits = BQ769x2_ReadAlarmStatus();

        if (AlarmBits & 0x80)
        {
            // FULLSCAN complete: new measurements available
            BQ769x2_ReadAllVoltages();
            Pack_Current = BQ769x2_ReadCurrent();
            Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
            Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);

            // Clear the FULLSCAN bit
            DirectCommands(AlarmStatus, 0x0080, W);
        }

        if (AlarmBits & 0xC000)
        {
            // Safety Status bits present in AlarmStatus
            BQ769x2_ReadSafetyStatus();

            if (ProtectionsTriggered & 1)
            {
                P1OUT |= BIT0; // Turn on LED to indicate protection
            }

            // Clear Safety Status Alarm bits
            DirectCommands(AlarmStatus, 0xF800, W);
        }
        else
        {
            if (ProtectionsTriggered & 1)
            {
                BQ769x2_ReadSafetyStatus();
                if (!(ProtectionsTriggered & 1))
                {
                    P1OUT &= ~BIT0; // Turn off LED when protection clears
                    printf("PROTECT");
                }
            }
        }

        // Output for every 20 cycles
        if (i % 20 == 0){
            printf("Running\n\r");
            output_bms_voltages(Stack_Voltage, CellVoltage, NUM_CELLS);
            update_fets_from_buttons();
            BQ769x2_ReadFETStatus();
            output_fet_status(CHG, DSG);
            output_current_value((int16_t)BQ769x2_ReadCurrent());
        }

        // Output for every 100 cycles
        if (i > 100){
            output_bms_config("Li-ion", 3);
            BQ769x2_ReadSafetyStatus();
            if (UV_Fault || OV_Fault || SCD_Fault || OCD_Fault){
                printf("Fault detected\n\r");
                printf("UV: %d", UV_Fault);
                printf("OV: %d", OV_Fault);
                printf("SCD: %d", SCD_Fault);
                printf("OCD: %d", OCD_Fault);
            }       
            i = 0;     
        }

        i++;
        delayUS(20000); // repeat loop every ~20 ms
    }
}

//------------------------------------------------------------------------------
// UART Output helper functions
//------------------------------------------------------------------------------

// Low level formatting function
static void send_json_notification_data(const char *method, const char *id, const char *data_str)
{
    uint32_t timestamp = 5005;

    printf("{\"method\":\"%s\","
           "\"data\":{%s},"
           "\"timestamp\":%lu,"
           "\"source\":\"robot\","
           "\"type\":\"notification\","
           "\"id\":\"%s\","
           "\"agent\":\"robot\"}\n",
           method, data_str, (unsigned long)timestamp, id);
}

// For simple, fixed data payloads
static void send_json_notification_fmt(const char *method,
                                       const char *id,
                                       const char *data_fmt, ...)
{
    char data_buf[256];
    va_list args;
    va_start(args, data_fmt);
    vsnprintf(data_buf, sizeof(data_buf), data_fmt, args); // format into data_buf
    va_end(args);

    send_json_notification_data(method, id, data_buf);
}

// -----------------------------------------------------------------------------
// Public “user” functions (no JSON/quote escaping here)
// -----------------------------------------------------------------------------

// 1. BMS/config  --------------------------------------------------------------

void output_bms_config(const char *battery_type, int cell_count)
{
    const char* type = "battery_type";
    const char* count = "cell_count";
    if (output_mode == OUTPUT_HUMAN)
    {
        printf("BMS config:\n\r");
        printf("  %s: %s\n\r", type, battery_type);
        printf("  %s: %d\n\r", count, cell_count);
    }
    else
    {
        // all escaping handled inside send_json_notification_fmt()
        send_json_notification_fmt(
            "/BMS/config",
            "001",
            "\"%s\":\"%s\",\"%s\":%d",
            type, battery_type, count, cell_count);
    }
}

// 2. Pack + cell voltages  ----------------------------------------------------
// v_mV: pack voltage in millivolts
// CellVoltage[i]: per-cell voltage in millivolts

void output_bms_voltages(uint32_t v_mV, const uint16_t *CellVoltage, int cell_count)
{
    if (output_mode == OUTPUT_HUMAN)
    {
        printf("Pack voltage: %u.%03u V\n\r",
               v_mV / 1000, v_mV % 1000);

        for (int i = 0; i < cell_count; i++)
        {
            printf("    Battery %d voltage: %u.%03u V\n\r",
                   i + 1,
                   CellVoltage[i] / 1000,
                   CellVoltage[i] % 1000);
        }
    }
    else
    {
        // build dynamic JSON data body, then wrap with send_json_notification_data()
        char data_buf[256];
        int len = 0;

        // pack voltage
        len += snprintf(data_buf + len, sizeof(data_buf) - len,
                        "\"pack_voltage_V\":%u.%03u",
                        v_mV / 1000, v_mV % 1000);

        // cell voltages
        for (int i = 0; i < cell_count && len < (int)sizeof(data_buf); i++)
        {
            len += snprintf(data_buf + len, sizeof(data_buf) - len,
                            ",\"cell%d_voltage_V\":%u.%03u",
                            i + 1,
                            CellVoltage[i] / 1000,
                            CellVoltage[i] % 1000);
        }

        send_json_notification_data("/BMS/voltages",
                                    "002",
                                    data_buf);
    }
}

// Example function here
void output_fet_status(bool chg, bool dsg)
{
    const char* chgStatus = "CHG_Status";
    const char* dsgStatus = "DSG_Status";
    if (output_mode == OUTPUT_HUMAN)
    {
        printf("%s: %d  %s: %d \n\r", chgStatus, chg, dsgStatus, dsg);
    }
    else
    {
        // all escaping handled inside send_json_notification_fmt()
        send_json_notification_fmt(
            "/BMS/fet_status",
            "003",
            "\"%s\":%d,\"%s\":%d",
            chgStatus, chg, dsgStatus, dsg);
    }
}

void output_error_notifications(bool UV, bool OV, bool SCD, bool OCD)
{
    const char* UV_s = "Under_Voltage_Status";
    const char* OV_s = "Over_Voltage_Status";
    const char* SCD_s = "SCD_Status";
    const char* OCD_s = "OCD_Status";
    if (output_mode == OUTPUT_HUMAN)
    {
        printf("%s: %d  %s: %d  %s: %d  %s: %d \n\r", UV_s, UV, OV_s, OV, SCD_s, SCD, OCD_s, OCD);
    }
    else
    {
        // all escaping handled inside send_json_notification_fmt()
        send_json_notification_fmt(
            "/BMS/error_notifications",
            "004",
            "\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d",
            UV_s, UV, OV_s, OV, SCD_s, SCD, OCD_s, OCD);
    }
}

void output_current_value(int16_t current_mA)
{
    const char* current_s = "Current_mA";
    if (output_mode == OUTPUT_HUMAN)
    {
        printf("%s: %d \n\r", current_s, current_mA);
    }
    else
    {
        // all escaping handled inside send_json_notification_fmt()
        send_json_notification_fmt(
            "/BMS/current_measurement",
            "005",
            "\"%s\":%d",
            current_s, current_mA);
    }
}



//------------------------------------------------------------------------------
// High-level BQ769x2 helpers
//------------------------------------------------------------------------------

void BQ769x2_Init(void)
{
    // Enter CONFIGUPDATE mode (Subcommand 0x0090)
    CommandSubcommands(SET_CFGUPDATE);

    // Power configuration
    BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);

    // Enable REG0 pre-regulator
    BQ769x2_SetRegister(REG0Config, 0x01, 1);

    // Enable REG1 with 3.3 V output
    BQ769x2_SetRegister(REG12Config, 0x0D, 1);

    // DFETOFF pin controls both CHG and DSG FET
    BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);

    // ALERT pin configuration
    BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

    // TS1 measures cell temperature
    BQ769x2_SetRegister(TS1Config, 0x07, 1);

    // TS3 measures FET temperature
    BQ769x2_SetRegister(TS3Config, 0x0F, 1);

    // HDQ pin: no thermistor on EVM; set to 0
    BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);

    // VCell Mode: enable 16 cells
    BQ769x2_SetRegister(VCellMode, NUM_CELLS_BITS, 2);

    // Enabled Protections A: SCD, OCD1, OCC, COV, CUV
    BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);

    // Enabled Protections B: all enabled
    BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);

    // Default Alarm Mask
    BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);

    // Cell balancing configuration
    BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);

    // CUV Threshold
    BQ769x2_SetRegister(CUVThreshold, 0x31, 1);

    // COV Threshold
    BQ769x2_SetRegister(COVThreshold, 0x55, 1);

    // OCC Threshold
    BQ769x2_SetRegister(OCCThreshold, 0x05, 1);

    // OCD1 Threshold
    BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);

    // SCD Threshold
    BQ769x2_SetRegister(SCDThreshold, 0x05, 1);

    // SCD Delay
    BQ769x2_SetRegister(SCDDelay, 0x03, 1);

    // SCDL latch limit
    BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);

    // Exit CONFIGUPDATE mode
    CommandSubcommands(EXIT_CFGUPDATE);
}

uint16_t BQ769x2_ReadAlarmStatus(void)
{
    DirectCommands(AlarmStatus, 0x00, R);
    return (RX_data[1] * 256 + RX_data[0]);
}

void BQ769x2_ReadSafetyStatus(void)
{
    // Safety Status A
    DirectCommands(SafetyStatusA, 0x00, R);
    value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);

    UV_Fault = ((0x4 & RX_data[0]) >> 2);
    OV_Fault = ((0x8 & RX_data[0]) >> 3);
    SCD_Fault = ((0x8 & RX_data[1]) >> 3);
    OCD_Fault = ((0x2 & RX_data[1]) >> 1);

    // Safety Status B
    DirectCommands(SafetyStatusB, 0x00, R);
    value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);

    // Safety Status C
    DirectCommands(SafetyStatusC, 0x00, R);
    value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);

    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1)
    {
        ProtectionsTriggered = 1;
    }
    else
    {
        ProtectionsTriggered = 0;
    }
}

void BQ769x2_ReadPFStatus(void)
{
    DirectCommands(PFStatusA, 0x00, R);
    value_PFStatusA = (RX_data[1] * 256 + RX_data[0]);

    DirectCommands(PFStatusB, 0x00, R);
    value_PFStatusB = (RX_data[1] * 256 + RX_data[0]);

    DirectCommands(PFStatusC, 0x00, R);
    value_PFStatusC = (RX_data[1] * 256 + RX_data[0]);
}

void BQ769x2_ReadFETStatus(void)
{
    DirectCommands(FETStatus, 0x00, R);
    FET_Status = (RX_data[1] * 256 + RX_data[0]);

    DSG = ((0x4 & RX_data[0]) >> 2);
    CHG = (0x1 & RX_data[0]);
    PCHG = ((0x2 & RX_data[0]) >> 1);
    PDSG = ((0x8 & RX_data[0]) >> 3);
}

void BQ769x2_WriteFETStatus(bool DSG, bool CHG){
    uint8_t State = 0;

    // DSG control
    if (!DSG){
        State |= (1 << 0);  // set DSG_OFF bit = force off
    }

    // CHG control
    if (!CHG){
        State |= (1 << 2);  // set CHG_OFF bit = force off
    }

    Subcommands(FET_CONTROL, State, 1);   // or whatever your write call requires
}

void update_fets_from_buttons(void)
{
    bool dsg_enable = false;
    bool chg_enable = false;

    if (P3IN & BIT3) {
        dsg_enable = true;        // p3.3 high -> enable discharge
    }

    if (P3IN & BIT4) {
        chg_enable = true;        // p3.4 high -> enable charge
    }

    BQ769x2_WriteFETStatus(dsg_enable, chg_enable);   // apply fet control
}




void BQ769x2_ShutdownPin(void)
{
    P2OUT |= BIT3;
}

void BQ769x2_ReleaseShutdownPin(void)
{
    P2OUT &= ~BIT3;
}

uint16_t BQ769x2_ReadVoltage(uint8_t command)
{
    DirectCommands(command, 0x00, R);

    if (command >= Cell1Voltage && command <= Cell16Voltage)
    {
        // Cell 1–16: mV
        return (RX_data[1] * 256 + RX_data[0]);
    }
    else
    {
        // Stack / Pack / LD: 0.01 V units
        return 10 * (RX_data[1] * 256 + RX_data[0]);
    }
}

void BQ769x2_ReadAllVoltages(void)
{
    unsigned char x;
    int cellvoltageholder = Cell1Voltage;

    for (x = 0; x < 16; x++)
    {
        CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
        cellvoltageholder += 2;
    }

    Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
    Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
    LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

uint16_t BQ769x2_ReadCurrent(void)
{
    DirectCommands(CC2Current, 0x00, R);
    return (RX_data[1] * 256 + RX_data[0]); // mA
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);
    return (0.1f * (float)(RX_data[1] * 256 + RX_data[0])) - 273.15f;
}

//------------------------------------------------------------------------------
// Low-level BQ769x2 register / command helpers
//------------------------------------------------------------------------------

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2] = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff; //1st byte of data

    switch(datalen)
    {
        case 1: //1 byte datalength
            I2C_WriteReg(0x3E, TX_RegData, 3);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
        case 2: //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 4);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
        case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 6);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
    }
}

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{   //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    I2C_WriteReg(0x3E,TX_Reg,2);
    delayUS(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R) {//read
        I2C_WriteReg(0x3E,TX_Reg,2);
        delayUS(2000);
        I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
    }
    else if (type == W) {
        //FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        I2C_WriteReg(0x3E,TX_Reg,3);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05; //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
    else if (type == W2){ //write data with 2 bytes
        //CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        I2C_WriteReg(0x3E,TX_Reg,4);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06; //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{   //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {//Read
        I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
        delayUS(2000);
    }
    if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command,TX_data,2);
        delayUS(2000);
    }
}

//------------------------------------------------------------------------------
// Utility functions
//------------------------------------------------------------------------------

void delayUS(uint16_t us)
{
    uint16_t ms = us / 1000;
    for (uint16_t i = 0; i < ms; i++)
    {
        __delay_cycles(1000);
    }
}

int putchar(int c)
{
    while (!(UCA1IFG & UCTXIFG))
        ;
    UCA1TXBUF = (unsigned char)c;
    return c;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        dest[i] = source[i];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
{
    unsigned char checksum = 0;

    for (unsigned char i = 0; i < len; i++)
    {
        checksum += ptr[i];
    }

    checksum = 0xff & ~checksum;
    return checksum;
}

//------------------------------------------------------------------------------
// I2C register access
//------------------------------------------------------------------------------

I2C_Mode I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize device address and interrupts */
    UCB1I2CSA = DEV_ADDR;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);        // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                      // Disable RX interrupt
    UCB1IE |= UCTXIE;                       // Enable TX interrupt
    UCB1CTLW0 |= UCTR + UCTXSTT;            // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
    //For debugger
    __no_operation();

    /* Copy over received data */
    CopyArray(ReceiveBuffer, reg_data, count);

    return MasterMode;
}

I2C_Mode I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    /* Copy register data to TransmitBuffer */
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize device address and interrupts */
    UCB1I2CSA = DEV_ADDR;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);        // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                      // Disable RX interrupt
    UCB1IE |= UCTXIE;                       // Enable TX interrupt
    UCB1CTLW0 |= UCTR + UCTXSTT;            // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
    //For debugger
    __no_operation();

    return MasterMode;
}

//------------------------------------------------------------------------------
// Board support: GPIO, I2C, UART, port mapping
//------------------------------------------------------------------------------

void LED_On(int32_t ledPin)
{
    assert(ledPin >= 0 && ledPin <= 2);
    P8OUT &= ~(1 << ledPin);
}

void LED_Off(int32_t ledPin)
{
    assert(ledPin >= 0 && ledPin <= 2);
    P8OUT |= (1 << ledPin);
}

void GPIO_initPins(void)
{
    P1OUT = 0x00;
    P2OUT = 0x00;
    P3OUT = 0x00;
    P4OUT = 0x0F;
    P5OUT = 0x00;
    P6OUT = 0x00;
    P8OUT = 0x00;

    P1DIR = 0xFF;
    P2DIR = 0xFF;
    P3DIR = 0xFF;
    P4DIR = 0x00;
    P5DIR = 0xFF;
    P6DIR = 0xFF;
    P8DIR = 0xFF;

    GPIO_configPins();
}

void GPIO_configPins(void)
{
    // P8.0 (LED1)
    P8OUT &= ~BIT0;                 // Set P8.0 to low
    P8DIR |= BIT0;                  // Set P8.0 to output direction

    // P8.1 (LED2)
    P8OUT |= BIT1;                  // Set P8.1 to high
    P8DIR |= BIT1;                  // Set P8.1 to output direction

    // P8.2 (LED3)
    P8OUT |= BIT2;                  // Set P8.2 to high
    P8DIR |= BIT2;                  // Set P8.2 to output direction

    P3DIR &= ~(BIT3 | BIT4);        // Set P4.3 and P4.4 to input
    P3REN |= (BIT3 | BIT4);         // Enable pull up/down resistor
    P3OUT &= ~(BIT3 | BIT4);        // Output low (pull down)
}

void map_I2C_and_UART(void)
{
    PMAPKEYID = PMAPKEY; // Unlock port mapping 
    PMAPCTL |= PMAPRECFG;

    P4MAP4 = PM_UCB1SCL; // Set P4.4 as SCL
    P4MAP5 = PM_UCB1SDA; // Set P4.5 as SCL

    P4MAP2 = PM_UCA1RXD; // Set 4.2 as RXD
    P4MAP3 = PM_UCA1TXD; // Set 4.3 as TXD

    PMAPKEYID = 0;
}

void I2C_initModule(void)
{
    // --- Configure I2C pins for UCB1 on MSP430F5529 ---
    // P4.4 = UCB1SDA, P4.5 = UCB1SCL
    P4SEL |= BIT4 | BIT5;      // select peripheral function for P4.4/P4.5
    //P4SEL |= BIT6 | BIT7;      // select peripheral function for P4.6/P4.7

    // --- Configure USCI_B1 in I2C master mode ---
    UCB1CTL1 |= UCSWRST;       // hold USCI in reset

    // CTL0: sync, I2C, master
    UCB1CTL0 = UCMST       |   // master mode
               UCMODE_3    |   // I2C mode
               UCSYNC;         // synchronous mode

    // CTL1: SMCLK as source while still in reset
    UCB1CTL1 = UCSWRST | UCSSEL_2;   // UCSSEL_2 = SMCLK

    // Baud rate: BRCLK / (UCB1BR0+256*BR1)
    // For example: SMCLK = 16 MHz, BR=40 -> 400 kHz I2C
    // We are running now without FLL (1Mhz)
    UCB1BR0 = 10;
    UCB1BR1 = 0;

    // 7-bit slave address of BQ769x2 (0x08) – goes into UCB1I2CSA
    UCB1I2CSA = DEV_ADDR;

    // Release from reset and start operation
    UCB1CTL1 &= ~UCSWRST;

    // Enable NACK interrupt (optional – your ISR already handles NACK)
    UCB1IE |= UCNACKIE;
}

void UART_Init(void)
{
    P4SEL |= BIT2 | BIT3;
    P4DIR |= BIT3;
    P4DIR &= ~BIT2;

    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 &= ~(UCSSEL_3);
    UCA1CTL0 = UCMODE_0;

    UCA1CTL1 |= UCSSEL_1; // ACLK
    UCA1BR0 = 0x03;
    UCA1BR1 = 0x00;
    UCA1MCTL = UCBRS_3 + UCBRF_0;

    UCA1CTL1 &= ~UCSWRST;
}

//------------------------------------------------------------------------------
// I2C ISR
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR(void)
#else
#error Compiler not supported!
#endif
{
    uint8_t rx_val = 0;

    switch (UCB1IV)
    {
    case USCI_NONE:
        break;

    case USCI_I2C_UCALIFG:
        break;

    case USCI_I2C_UCNACKIFG:
        break;

    case USCI_I2C_UCSTTIFG:
        break;

    case USCI_I2C_UCSTPIFG:
        break;

    case USCI_I2C_UCRXIFG3:
    case USCI_I2C_UCTXIFG3:
    case USCI_I2C_UCRXIFG2:
    case USCI_I2C_UCTXIFG2:
    case USCI_I2C_UCRXIFG1:
    case USCI_I2C_UCTXIFG1:
        break;

    case USCI_I2C_UCRXIFG0:
        rx_val = UCB1RXBUF;
        if (RXByteCtr)
        {
            ReceiveBuffer[ReceiveIndex++] = rx_val;
            RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
            UCB1CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
            UCB1IE &= ~UCRXIE;
            MasterMode = IDLE_MODE;
            __bic_SR_register_on_exit(CPUOFF);
        }
        break;

    case USCI_I2C_UCTXIFG0:
        switch (MasterMode)
        {
        case TX_REG_ADDRESS_MODE:
            UCB1TXBUF = TransmitRegAddr;
            if (RXByteCtr)
            {
                MasterMode = SWITCH_TO_RX_MODE;
            }
            else
            {
                MasterMode = TX_DATA_MODE;
            }
            break;

        case SWITCH_TO_RX_MODE:
            UCB1IE |= UCRXIE;
            UCB1IE &= ~UCTXIE;
            UCB1CTLW0 &= ~UCTR;
            MasterMode = RX_DATA_MODE;
            UCB1CTLW0 |= UCTXSTT;

            if (RXByteCtr == 1)
            {
                while (UCB1CTLW0 & UCTXSTT)
                    ;
                UCB1CTLW0 |= UCTXSTP;
            }
            break;

        case TX_DATA_MODE:
            if (TXByteCtr)
            {
                UCB1TXBUF = TransmitBuffer[TransmitIndex++];
                TXByteCtr--;
            }
            else
            {
                UCB1CTLW0 |= UCTXSTP;
                MasterMode = IDLE_MODE;
                UCB1IE &= ~UCTXIE;
                __bic_SR_register_on_exit(CPUOFF);
            }
            break;

        default:
            __no_operation();
            break;
        }
        break;

    default:
        break;
    }
}

#endif
