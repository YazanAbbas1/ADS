# ADS (Angular displacement sensor from bendlab)
Interfacing an ADS one-axis with an ESP32-DevKitC.
---
According to the porting documentation provided by bendlab, only one file (ads_hal_i2c.c) should be implemented based on which MCU we use.
In this case, the code is compatible and tested on an ESP32 MCU.

Despite that the documentation says that only one file should be modified, I implemented a couple of functions in the file ads.c to make it easier to use the code.
The API published by bendlabs can be downloaded form the following [repo](https://github.com/bendlabs/one_axis_ads/tree/master/portable/).
---
## Functions added in the ads.c file:
Only two new functions added to this file and they are marked with a comment that says "new code added". You can copy this two functions and paste them in your ads.c file. OBS! don't forget to add the prototypes of the new functions in the header file (ads.h) which also are marked in the code as "new added code".
## Functions implemented in the ads_hal_i2c.c:
all the functions that are mentioned in the porting guide provided by bendlabs and existing under the following [link](https://github.com/bendlabs/one_axis_ads/tree/master/documentation/), are implemented.
---
## How to use the code:
* To initialize the sensor call the function ads() in the main function.
* To read the sensor's data you need to call the function ads_read().
