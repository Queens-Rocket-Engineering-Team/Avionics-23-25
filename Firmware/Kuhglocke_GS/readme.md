# Kuhglocke Firmware

This is firmware for the QRET Kuhglocke portable ground station (Rev 1.0). As of writing, this firmware was used at the Launch Canada 2024 competition. The firmware is effectively in a demo-draft state, and certain functionality (such as buttons, menus, speaker output, etc) have not been implemented.

We used the "esp32" core by Espressive Systems (working with v2.0.5), and the "ESP32 Sketch Data Upload" plugin for flashing static webpage data to the ESP32's flash. Kuhglocke features an automatic programming circuit and should not need any BOOT or RESET buttons to be pressed during firmware flashing over USB.