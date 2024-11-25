# Kuhglocke Portable Ground Station

This directory contains the V1.0 revision of the Kuhglocke ground station. It has several known problems;

### Critical Problems
- VSS/VDD pins are reversed on the SD card. This results in immediately destroying the MicroSD card. Swap pins to fix.
- NAU7802 ADC range is half of expected; change R17 to 20k to allow battery measurement.
- The CP2012's reset pullUP resistor (R29) is erroneously connected to GND, prevents it from functioning. Change to pullup to fix.
- The 5V disabling the Buck-Boost PSU causes brownouts (circuit around Q6); redesign circuit or remove function to fix.

### Future Suggestions
- YIC31616 GPS module should have a V_BCKP source (TTFF is very slow)
- The NAU7802 ADC is slow & difficult to work with; switch to something else
- Look into using the MAX17043 for battery management
- Look into using TP4056 for battery charging (cheaper)