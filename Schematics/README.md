# Schematics

This directory contains all schematics involved in this iteration of the Avionics system.

Schematics are made using KiCAD 7.

The `Templates` directory contains a KiCAD project with miscellaneous templates, such as the stack edge connector and template module PCB layout.

## Backplane
- Single large PCB
- Connects all stack "modules" (PCBs)
- Allows communication between modules (CANBUS)
- Cleaner routing for power

## GPS Module
- Acquires GPS signal
- Logs GPS data locally
- Has a daughter-board for the GPS receiver

## Communications Module
- Tracks data from other modules using CANBUS
- Remotely transmits data to ground station

## Altimter Module
- Tracks altitude using pressure and acceleration
- Deploys parachute charges
- Runs on secondary power supply rail
- Logs altitude, pressure, events, etc. locally

## Sensors Module
- Has various on-board and external sensors
- Logs data from above sensors locally

## Power Module
- Provides power to all other modules
	- Primary and Secondary 3.3V rails
	- Primary 5.0V rail
- Facilitates connections between backplane and external devices
	- Batteries
	- Screw switches