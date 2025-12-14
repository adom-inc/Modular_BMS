# Modular BMS Repository

This repository contains hardware and firmware for a **Modular Battery Management System (BMS)**.

## Notes

To reset the system, please power cycle the MSP Ez-FET Lite Molecule which powers the MSP onboard the BQ76952EVM. 
If for some reason the BQ76952 chip itself is stuck, you may pull the "RST_SHUT" line high (or to REG1)
A minimum of 3 batteries are required

## Firmware Documentation
Detailed firmware documentation can be found in:

```
Modular_BMS/PlatformIO/MSP430_BQ76952/src/README.md
```

This README covers firmware architecture, configuration, and usage for the MSP430 + BQ76952 system.

## PlatformIO Setup
Instructions for setting up the PlatformIO development environment are located in:

```
Modular_BMS/PlatformIO/README.md
```

Please follow that guide before attempting to build or flash the firmware.
