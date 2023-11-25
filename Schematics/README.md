# Schematics

This directory contains all schematics involved in this iteration of the Avionics system.

## Backplane
- Single, large PCB
- Connects all stack "modules" (PCBs)
- Allows communication between modules
- Cleaner routing for power

## GPS Module
- Acquires GPS signal
- Logs GPS data locally

## Communications Module
- Tracks data from other modules using CANBUS
- Remotely transmits data to ground station

## Altimter Module
- Tracks altitude using pressure and accelerometer
- Deploys parachute charges
- Logs altitude, pressure, events, etc. locally

## Sensors Module
- Has various on-board sensors
- Logs data from above sensors locally

## Power Module
- Facilitates connections between backplane and external devices
	- Batteries
	- Screw switches