# ThermoBoard DAQ Upgrade Software

This is the software for APRL's Thermocouple DAQ Board.

## Status

The codebase currently follows the following repo to implement LwIP since it is not supposed OOTB by STM32Cube: https://community.st.com/t5/stm32-mcus/how-to-use-the-lwip-ethernet-middleware-on-the-stm32h5-series/ta-p/691100#toc-hId--1862973319

It currently follows their example, with LED pin PC7 to represent link status. DHCP is currently disabled, but may be enabled in the future as needed.

Currently, CMake is used.

## Usage Notes

### Error states
During a sensor fault, the firmware will simply return a reading of 0.0f for that thermocouple. It will keep trying to clear the state during every supposed reading, which will likely make the fault LED blink under normal operating conditions.

RCC is enabled in case of HSE crystal failure.

### LEDs
| LED | Usage |
|-----|-----|
| PC7 | Link status |
| PC8 | Data sent over ethernet |
| PC9 | UDP send failure |

### Lead resistance
Theoretically the per-lead resistance maximum of the MAX31856 is 40k. Currently, it is set in firmware to trigger in less than 2 ms, with a lead resistance greater than 5k. If lead resistance is less than 5k, a fault state may be triggered.

### Sample Rate
Currently, the sample rate per sensor is set to ~>5Hz. This achieves the desired overall sample rate of 20Hz. You can calculate it with 1000ms / (90ms + (AVG_TC_SAMPLES - 1) + 33.33). 

### Thermocouple Type
This board can accomodate any type of thermocouple you could ever want.

## TODO

| Feature | Implemented? | Needed for hotfire? | Notes |
|---------|--------------|---------------------|-------|
| CLK | Yes | Yes | 25 MHz HSE w/RCC. See ThermoSoft.ioc for frequencies set. |
| ETH | Yes | Yes | Still needs to be piped in. |
| MAX31856 | Yes | Yes | Library imported, needs to be packed into a struct and sent over ETH. |
| CAN-FD | No | No | May not be implemented at this time. |
| FMAC | No | No | Enabled but may not be needed. Useful for hardware-accelered FIR filters. |
| CI | Yes | No | Just a simple CMake build, shouldn't be too hard. |
| Formatting | Yes | No | clang-format with GCC style would likely fit pretty well. |

### Additional Notes
Currently the chip itself does some basic supersampling. To improve sample rate, however, it may be a good idea to have the sensor send data at every possible opportunity that it can, then doing an actual true FIR filter on the H5. The FMAC is enabled on this chip just in case, however, the FMAC is only capable of doing fixed-point math, and the MAX31856 returns floating point (which isn't actually too computationally expensive to convert between). With this, we can achieve ~11.11Hz per sensor.