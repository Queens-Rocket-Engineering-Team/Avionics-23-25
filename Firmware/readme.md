# SRAD Firmware

This directory contains the firmware for SRAD modules (all V1.0 revisions). As of writing, this firmware was used during the Launch Canada 2024 competition. The code was originally forked from the `Spaceport_Firmware` branch, which contains firmware flown at the Spaceport America Cup 2024 competition (which has some known bugs). The module firmware in this directory should be mostly functional.

We currently use the Arduino IDE with the STM32Duino core ("STM32 MCU based boards" by STMicroelectronics, working with 2.7.1) for developing firmware for our modules (working with ). We flash firmware to the modules using SWD via an STLinkV2.

This directory also contains the firmware for the Kuhglocke ground station (revision V1.0). That firmware is developed using the Arduino IDE and the ESP32 core. We use the "esp32" core by Espressive Systems (working with v2.0.5). We flash the firmware to the ground station over USB. Static data (such as webpages) are uploaded using the "ESP32 Sketch Data Upload" plugin.

## Attributions

Our firmware uses a variety of external libraries. Most libraries can be found through the Arduino library manager, and those that cannot have the corresponding GitHub link beside their include statements. We have taken care not to include any external libraries within our repository; only code we have written (or have modified and credited) are included here.

- QMA6100P library was written by Tristan (ft. Brent)
- The RDES compression & FlashTable system was developed by Kennan. [See RDES repository.](https://github.com/Kenneract/Realtime-Deviation-Encoding-Scheme)
- The modified NAU7802_2CH library was written by Sparkfun & forked by Kennan [See NAU7802_2CH repository.](https://github.com/Kenneract/NAU7802_Arduino_2CH)