# ThermoBoard DAQ Upgrade Software

This is the software for APRL's Thermocouple DAQ Board.

## Status

The codebase currently follows the following repo to implement LwIP since it is not supposed OOTB by STM32Cube: https://community.st.com/t5/stm32-mcus/how-to-use-the-lwip-ethernet-middleware-on-the-stm32h5-series/ta-p/691100#toc-hId--1862973319

It currently follows their example, with LED pin PC7 to represent link status. DHCP is currently disabled, but may be enabled in the future as needed.

Currently, CMAKE is used.


## TODO
| Feature | Implemented? | Needed for hotfire? | Notes |
|---------|--------------|---------------------|-------|
| CLK | Yes | Yes | 25 MHz HSE w/RCC. See ThermoSoft.ioc for frequencies set. |
| ETH | Partial | Yes | Still needs to be piped in. |
| MAX31856 | No | Yes | Library imported, needs to be packed into a struct and sent over ETH. Needs to be forked for the H5. |
| CAN-FD | No | No | May not be implemented at this time. |
| FMAC | No | No | Enabled but may not be needed. Useful for hardware-accelered FIR filters. |
| CI | No | No | Just a simple CMake build, shouldn't be too hard. |
| Formatting | No | No | clang-format with GCC style would likely fit pretty well. |
