# Rocket Live Telemetry Overlay
**Queen's University Rocket Engineering Team**

## Overview
Real-time telemetry overlay for rocket launches. Currently uses fake data for testing - needs to be connected to actual rocket telemetry.

## Dependencies
```bash
pip install opencv-python numpy
```

## Usage
```bash
python overlay.py
```
Press `ESC` or `q` to quit.

## Connecting to Real Rocket Data

### What to Change
Replace the `get_live_data()` function in `overlay.py` with your actual telemetry code.

### Example - Serial Connection
```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port/baud rate

def get_live_data(t0: float) -> Dict:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            # Parse your data format here
            data = parse_your_packet(line)
            
            return {
                "mission_time": data.get('time', 0.0),
                "altitude": data.get('alt', 0.0),
                "velocity": data.get('vel', 0.0),
                "acceleration": data.get('acc', 0.0),
                "latitude": data.get('lat', 0.0),
                "longitude": data.get('lon', 0.0),
                "pressure": data.get('pressure', 101.3),
                "battery": data.get('voltage', 12.0),
                "int_temp": data.get('temp', 25.0),
                "video_conn_ok": True,  # Set based on your connection status
                "parachute_ready": data.get('chute_ready', False),
                "parachute_deployed": data.get('chute_deployed', False),
                "current_stage": determine_stage(data),  # Your stage logic
                "orientation": {
                    "pitch": data.get('pitch', 0.0),
                    "roll": data.get('roll', 0.0),
                    "yaw": data.get('yaw', 0.0)
                }
            }
    except Exception as e:
        print(f"Error reading telemetry: {e}")
        return get_fallback_data()  # Return safe defaults
```

### Notes
- Adjust the data parsing to match your rocket's packet format
- Update the flight stage logic based on your mission profile
- Add error handling for connection losses
- Test with simulated data first

## Troubleshooting
- **No camera detected**: Make sure your webcam is connected and not used by another application
- **Serial port issues**: Check permissions on Linux (`sudo usermod -a -G dialout $USER`)
- **High CPU usage**: Reduce frame rate or resolution in camera settings
- **Data parsing errors**: Add more robust error handling and data validation

## Future Improvements
- [ ] Data logging to file
- [ ] Replay mode for post-flight analysis  
- [ ] Multiple camera support
- [ ] Web-based remote monitoring
- [ ] Alert system for critical values

---
