# BHI260AP/BHI260AB/BHA260AB Sensor API, ported to Nordic nRF Connect SDK

Orignal code by Bosch (https://github.com/BoschSensortec/BHY2-Sensor-API), ported to the Nordic nRF Connect SDK version 2.0.1. Has been tested on a nRF5340DK, starting from the Blinky application available.

BHY2* libraries are unchanged, the only ported portions are common.c/h and the quaternion.c example.

If you are using **nRF5 SDK** insted, please have a look at https://github.com/robcazzaro/nRF52-BHY2-Sensor-API

# BHI260AP - nRF53DK Connection

BHI260AB connected to BMM150 via I2C, as the Shuttle board. In fact, it has been tested using the shuttle board itself.

For the DK, every PIN used is the default one so:
- **I2C SDA**: P1.02
- **I2C SCL**: P1.03

The nRF53DK Board needs to have both USB port connected, and be sure that SW9 (nRF POWER SOURCE) is set to USB. This way you will provide the correct voltage to the shuttle board.

# quaternion.c example

You can test the example starting from the Blinky application for nRF Connect SDK.
After having created the application, you need to:

- Update the CMakeLists.txt in order to target the BHY Library.
- Update thte prj.conf file as it follows:
```
    CONFIG_GPIO=y
    CONFIG_I2C=y
    CONFIG_CBPRINTF_FP_SUPPORT=y
```
- Create or update the devicetree overlay as it follows:
```
    &i2c1 {
    bhi260ap_sb: bhi260ap_sb@28{
        compatible = "i2c-device";
        reg = <0x28>;
        label = "BHI";

    };
};
```
The code itself should work on any SoC that can be flashed with nRF Connect. The configuration files, (prj.conf, dts etc) may vary from board to board.

## Original BOSCH Readme

> This package contains BH260AP/BHI260AB/BHA260AB generically clustered as BHy2 sensor API

Product links
- [BHA260AB](https://www.bosch-sensortec.com/products/smart-sensors/bha260ab.html)
- [BHI260AB](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab.html)
- [BHI260AP](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ap/)

---
#### Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved
