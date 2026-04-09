import requests
import platform #Used for testing as of now
import os #Used for testing as of now
import math
import time

def fetch_transmitter_data():
    """
    Fetches data from the transmitter's HTTP endpoint in a continuous loop.
    """

    # URL of the webpage hosted on the transmitter wifi network
    url = "http://192.168.4.1/debugData" #Comment out when testing without device >>>>>>

    try:
        #Sends a GET request to the transmitter
        #(connect timeout, read timeout)
        #make sure read timeout is smaller than camera refresh rate
        response = requests.get(url, timeout=(0.1,0.03)) 

        # Raises an exception if the HTTP response contains an error status code
        response.raise_for_status() 

        # Gets raw data as string
        raw_data = response.text 
        #raw_data = "212269,3,12,238.3,2375,3300,3941,76,205,Discharging,Power + Data,4294967295,0,0.000000,0.000000,0.00,1,No,?,?,?,1463.6,1430.0,905.40,31.25,10,212270,0000000000000000000000000000,0,0.00,0,NO,NO,0,0.000000,0.000000,0,0,0,8612,1"


        # Define the schema based on the provided GitHub code
        schema = [
            "Uptime", "MainLoopSpeed", "MaxMainLoopSpeed", "FreeHeap", "AmbientTemp",
            "PSUVolt", "BattVolt", "BattSoC", "SysCurrent", "BattStatus", "USBMode",
            "FixAge", "NumSat", "Lat", "Lon", "Alt", "Timestamp",
            "SDPresent", "SDCapacity", "SDAvailable", "LogID", "SPIFFSSize", "SPIFFSFree",
            "LoRaFreq", "LoRaBand", "LoRaSF", "LastPingTime", "LastPacket", "LastRSSI",
            "LastSNR", "LastFreqErr", "LastPcktValid", "AFCOn",
            "RcktSats", "RcktLat", "RcktLon", "RcktAlt", "RcktStatus",
            "CurFreqOffset", "Core0FreeStack", "Core0LoopTime"
        ]

        # Split the raw data string by commas 
        values = raw_data.split(',')

        # Map the schema to the values, N/A for things without values
        readable_data = {schema[i]: values[i] if i < len(values) else "N/A" for i in range(len(schema))}

    #Error handling
    except requests.exceptions.RequestException as e:
        print(f"Error fetching data: {e}")
        raise e
    
    return readable_data


#======================================================
# Velocity & Acceleration Calculations
#======================================================

def calculate_vertical_velocity(readable_data, previous_altitude, previous_uptime):
    """
    Calculates vertical velocity using altitude and time.
    """
    
    #Get altitude and uptime
    current_altitude = float(readable_data["Alt"])
    current_uptime = float(readable_data["Uptime"]) / 1000  # Convert ms to seconds

    if previous_altitude is not None and previous_uptime is not None:
        delta_h = current_altitude - previous_altitude
        delta_h_meters = delta_h * 0.304 #Convert from ft to m
        delta_t = current_uptime - previous_uptime  # Use Uptime for accurate telemetry timing

        vertical_velocity = delta_h / delta_h_meters if delta_h_meters > 0 else 0
    else:
        vertical_velocity = 0  # No previous values yet

    return vertical_velocity
    
    #return math.sin(27*time.time())


def calculate_vertical_acceleration(velocity, readable_data, previous_velocity, previous_uptime):
    """
    Calculates vertical acceleration using velocity and telemetry's Uptime.
    """
    
    # Get current velocity and telemetry time (Uptime converted from ms to sec)
    current_velocity = velocity
    current_uptime = float(readable_data["Uptime"]) / 1000  # Convert ms to seconds

    if previous_velocity is not None and previous_uptime is not None:
        delta_v = current_velocity - previous_velocity
        delta_t = current_uptime - previous_uptime  #Compare time from last velocity update

        vertical_acceleration = delta_v / delta_t if delta_t > 0 else 0
    else:
        vertical_acceleration = 0  #No previous values yet

    return vertical_acceleration
    
    #return math.cos(10*time.time())


#======================================================
# Display Function
#======================================================

def test_display_telemetry_data(readable_data):
    """
    Displays telemetry data in a readable format and clears the screen before each update.

    Args:
        parsed_data (dict): Parsed telemetry data as a dictionary.
    """

    # Clear the screen (works for both Windows and Linux)
    if platform.system() == "Windows":
        os.system("cls")  # Windows clear screen command
    else:
        os.system("clear")  # Linux/Mac clear screen command
    
    print("\nTelemetry Data (Live):")
    print("-" * 40)
    for key, value in readable_data.items():
        print(f"{key}: {value}")
    print("-" * 40)

if __name__ == '__main__':
    while True:
        print(fetch_transmitter_data())
        time.sleep(1)