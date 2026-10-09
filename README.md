# Arduino STS_Servos driver

[![arduino-library-badge](https://www.ardu-badge.com/badge/STS_Servos.svg?)](https://www.ardu-badge.com/STS_Servos)
![ci status badge](https://github.com/matthieuvigne/STS_Servos/actions/workflows/platformio-ci.yml/badge.svg)

<https://github.com/user-attachments/assets/4ff4030c-b8c4-4994-ae51-3bceac055187>

A library to drive [Feetech's STS smart servos](https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html) - in particular the STS 3215.
This low-cost servo offers very interesting properties at a quite cheap price (approx. $20 at 2024-10-05):

- Stall torque 19kg.cm
- Running speed 50rpm @ 7V.
- 360° servo mode - plus multiturn and step mode
- Open and close loop velocity commands
- Position, current and temperature sensors

Register access is private. Use named getters/setters; driver owns register widths,
encoding, value ranges and EEPROM turnaround timing. Torque limits use 0..1000;
position offsets use signed -2047..2047 counts. Acceleration accepts 0..254.

Setters return false on invalid values, communication failures or servo faults.
Unicast writes require response status level 1 (servo default); broadcast writes
cannot be acknowledged. Reads can return a legitimate zero: inspect getLastError()
to distinguish failure. Copy its message before another driver call, which clears it.
Keep each driver on one bus task, or serialize calls externally.

Run tests/Run-DriverCheck.ps1 in PowerShell with installed MSVC to check packet widths,
encoding, validation and communication failures without attached motors.

## Example

To debug STS servo control, official interface board, [FE-URT-1](https://www.feetechrc.com/FE-URT1-C001.html), is essential. You can change Servo ID with this board.

The I/F board has pins to communicate with a micro-controller. The following table shows sample connections between ESP32 and the I/F. [SimpleSweepWithInterfaceBoard.ino](./examples/SimpleSweepWithInterfaceBoard/SimpleSweepWithInterfaceBoard.ino) is a sample sketch to drive a STS servo. The video in this README is results of this sketch.

| ESP32   |  --   |  FE-URT-1  |
| :-----: | :---: | :--------: |
|   5V    |  --   |     5V     |
|   GND   |  --   |    GND     |
| 32 (RX) |  --   | RXD (Silk) |
| 26 (TX) |  --   | TXD (Silk) |
