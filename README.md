# ADS (Angular displacement sensor from bendlab)
Interfacing an ADS one-axies with an ESP32-DevKitC.
---
According to the porting documentation provided by bendlab, only one file (ads_hal_i2c.c) should be implemented based on which MCU we use.
In this case, the code is compatible and tested on an ESP32 MCU.
Despite that the documentation says that only one file should be implemented, I implemented a couple of functions in the file ads.c to make it easier to use the code.
The API published by bendlabs can be downloaded form the following [repo] (https://github.com/bendlabs/one_axis_ads/tree/master/portable).
---


