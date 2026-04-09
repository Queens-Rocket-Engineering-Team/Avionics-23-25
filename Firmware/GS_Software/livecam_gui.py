#!/usr/bin/env python3
"""
Queen's University Rocket Engineering Team – Live Telemetry Overlay

* Opens the first webcam and draws telemetry around the frame.
* Replace `get_live_data()` with real serial/GSE‑link code to feed actual telemetry.
* Uses only OpenCV drawing primitives so it stays portable.

Controls
========
• ESC or q – Quit

Dependencies
============
    pip install opencv-python numpy

"""

import cv2
import numpy as np
import time
import math
import random
from typing import Dict
from datetime import datetime

#for ground station communication
from fetch_transmitter import (
    fetch_transmitter_data,
    calculate_vertical_velocity,
    calculate_vertical_acceleration
)

# --------------------------- CONFIG ---------------------------
WINDOW_NAME = "Rocket Live Telemetry"
FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720
FONT         = cv2.FONT_HERSHEY_SIMPLEX

# Modern color palette
COLORS = {
    'primary_text': (255, 255, 255),      # White
    'secondary_text': (200, 200, 200),    # Light gray
    'ascent': (0, 170, 255),              # Bright blue
    'success': (0, 255, 150),             # Modern green
    'warning': (255, 190, 0),             # Amber
    'danger': (255, 80, 80),              # Modern red
    'background': (40, 40, 50),           # Dark blue-gray
    'surface': (60, 60, 70),              # Lighter surface
    'crosshair': (0, 255, 200),           # Cyan
    'progress_bg': (50, 50, 60),          # Dark progress background
}

# Value ranges (for bar scaling)
ALT_RANGE     = (0, 4000)     # metres
VEL_RANGE     = (-60, 300)    # m/s
ACC_RANGE     = (-20, 60)     # m/s²
BATT_RANGE    = (9.0, 13.0)   # volts
RSSI_RANGE    = (-90, -40)    # dBm (unused for now)

# Flight stages for progression indicator
FLIGHT_STAGES = ["Rail", "Ascent", "Drogue", "Main", "Landed", "Ballistic"]

# Read and store data from Kuhglocke
def get_live_data(t0: float, prev_data: dict) -> Dict:
    """Return a dictionary with live telemetry. Currently simulated."""
    t = time.time() - t0

    try:
        #read data from groud station
        raw_data = fetch_transmitter_data()

        #initialize some values to zero for first second to avoid error
        if t < 1:
            vert_vel = 0
            vert_accel = 0
        else:
            vert_vel = calculate_vertical_velocity(raw_data, prev_data['velocity'], prev_data['uptime'])
            vert_accel = calculate_vertical_acceleration(vert_vel, raw_data,  prev_data['velocity'], prev_data['uptime'])


        data = {
            "mission_time": t,
            "uptime": float(raw_data['Uptime']),
            "altitude": float(raw_data['Alt']), #VS RcktAlt
            "velocity": float(vert_vel),
            "acceleration": float(vert_accel),
            "latitude": float(raw_data['Lat']), #VS RcktLat
            "longitude": float(raw_data['Lon']), #VS RcktLon
            "pressure": 101.3 - float(raw_data['Alt'][-1]) * 0.012, # crude ISA model
            "battery": float(raw_data['BattVolt']), #CHECK WHICH BATTERY
            "int_temp": float(raw_data['AmbientTemp']),
            "video_conn_ok": True, #need to add error handling for when camera is offline and update this
            "current_stage": prev_data['current_stage'],
            "gs_connected":True
        }

        return data

    except Exception as e:
        #print(f"Error updating GUI: {e}")
        data = prev_data
        data['mission_time']=t
        data['gs_connected'] = False
        
        return data

# --------------------------- DRAW HELPERS ---------------------------

def draw_rounded_rect(img, top_left, width, height, color, radius=8, alpha=0.8):
    """Draw a rounded rectangle with transparency."""
    x, y = top_left
    overlay = img.copy()
    
    # Create rounded rectangle
    cv2.rectangle(overlay, (x + radius, y), (x + width - radius, y + height), color, -1)
    cv2.rectangle(overlay, (x, y + radius), (x + width, y + height - radius), color, -1)
    cv2.circle(overlay, (x + radius, y + radius), radius, color, -1)
    cv2.circle(overlay, (x + width - radius, y + radius), radius, color, -1)
    cv2.circle(overlay, (x + radius, y + height - radius), radius, color, -1)
    cv2.circle(overlay, (x + width - radius, y + height - radius), radius, color, -1)
    
    # Blend with original image
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_text(img, text, org, color=None, scale=0.6, thickness=1, shadow=True):
    """Draw text with optional shadow for better readability."""
    if color is None:
        color = COLORS['primary_text']
    
    if shadow:
        # Draw shadow
        shadow_org = (org[0] + 1, org[1] + 1)
        cv2.putText(img, text, shadow_org, FONT, scale, (20, 20, 20), thickness + 1, cv2.LINE_AA)
    
    # Draw main text
    cv2.putText(img, text, org, FONT, scale, color, thickness, cv2.LINE_AA)


def draw_modern_bar(img, label, value, vmin, vmax, top_left, length=180, height=16,
                   bar_color=None, show_value=True):
    """Draw a modern progress bar with rounded corners and gradient effect."""
    if bar_color is None:
        if value < (vmin + vmax) * 0.3:
            bar_color = COLORS['danger']
        elif value < (vmin + vmax) * 0.7:
            bar_color = COLORS['warning']
        else:
            bar_color = COLORS['success']
    
    x, y = top_left
    
    # Background rounded rectangle
    draw_rounded_rect(img, (x, y), length, height, COLORS['progress_bg'], radius=height//2, alpha=0.7)
    
    # Progress fill
    fill = max(0, min(1, (value - vmin) / (vmax - vmin)))
    fill_width = int(fill * length)
    
    if fill_width > 0:
        draw_rounded_rect(img, (x, y), fill_width, height, bar_color, radius=height//2, alpha=0.9)
    
    # Label and value
    if show_value:
        draw_text(img, f"{label}", (x, y - 8), color=COLORS['secondary_text'], scale=0.5)
        draw_text(img, f"{value:.1f}", (x + length + 10, y + height - 3), 
                 color=COLORS['primary_text'], scale=0.55)


def draw_modern_indicator(img, label, state, center, size=8):
    """Draw a modern status indicator with glow effect."""
    color = COLORS['success'] if state else COLORS['danger']
    x, y = center
    
    # Outer glow
    cv2.circle(img, center, size + 2, (color[0]//3, color[1]//3, color[2]//3), -1)
    # Main circle
    cv2.circle(img, center, size, color, -1)
    # Inner highlight
    cv2.circle(img, (x - 2, y - 2), size//3, (255, 255, 255), -1)
    
    draw_text(img, label, (x + size + 8, y + 5), color=COLORS['primary_text'], scale=0.5)


def draw_info_panel(img, title, items, top_left, width=300, item_height=25):
    """Draw a modern info panel with background."""
    x, y = top_left
    panel_height = len(items) * item_height + 40
    
    # Background panel
    draw_rounded_rect(img, (x, y), width, panel_height, COLORS['surface'], alpha=0.85)
    
    # Title
    draw_text(img, title, (x + 15, y + 25), color=COLORS['ascent'], scale=0.65, thickness=2)
    
    # Items
    for i, item in enumerate(items):
        item_y = y + 50 + i * item_height
        draw_text(img, item, (x + 15, item_y), color=COLORS['primary_text'], scale=0.55)


def draw_stage_progress(img, current_stage, top_left, width=450, height=24):
    """Draw modern flight stage progression indicator."""
    x, y = top_left
    
    # Background
    draw_rounded_rect(img, (x, y), width, height, COLORS['progress_bg'], alpha=0.8)
    
    # Progress fill with gradient
    progress = (current_stage+1) / (len(FLIGHT_STAGES)-1)
    fill_width = int(progress * width)
    
    if fill_width > 0:
        draw_rounded_rect(img, (x, y), fill_width, height, COLORS['ascent'], alpha=0.9)
    
    # Stage markers
    for i in range(len(FLIGHT_STAGES)):
        marker_x = x + int((i / ((len(FLIGHT_STAGES)) - 1)) * width)
        cv2.line(img, (marker_x, y + 2), (marker_x, y + height - 2), COLORS['secondary_text'], 2)
    
    # Current stage indicator (modern triangle)
    indicator_x = x + int(progress * width)
    points = np.array([[indicator_x, y - 10], [indicator_x - 8, y], [indicator_x + 8, y]], np.int32)
    cv2.fillPoly(img, [points], COLORS['warning'])
    cv2.polylines(img, [points], True, COLORS['primary_text'], 1)
    
    # Current stage text
    draw_text(img, f"STAGE: {FLIGHT_STAGES[current_stage].upper()}", 
             (x, y - 20), color=COLORS['ascent'], scale=0.6, thickness=2)


def draw_crosshair(img, center, size=20, color=None, thickness=2):
    """Draw modern crosshair with subtle styling."""
    if color is None:
        color = COLORS['crosshair']
    
    x, y = center
    gap = 4
    
    # Horizontal lines
    cv2.line(img, (x - size, y), (x - gap, y), color, thickness)
    cv2.line(img, (x + gap, y), (x + size, y), color, thickness)
    
    # Vertical lines
    cv2.line(img, (x, y - size), (x, y - gap), color, thickness)
    cv2.line(img, (x, y + gap), (x, y + size), color, thickness)

def draw_logo(img, logo, margin=10):
    #find positions
    lh, lw = logo.shape[:2]
    ih, iw = img.shape[:2]
    y1 = ih - lh - margin
    y2 = y1 + lh
    x1 = iw - lw - margin
    x2 = x1 + lw

    #check bounds
    if y1 < 0 or x1 < 0:
        return

    #blend alpha channel
    if logo.shape[2] == 4:
        alpha_logo = logo[:, :, 3] / 255.0
        for c in range(3):
            img[y1:y2, x1:x2, c] = (
                alpha_logo * logo[:, :, c] +
                (1 - alpha_logo) * img[y1:y2, x1:x2, c]
            ).astype(np.uint8)
    else:
        img[y1:y2, x1:x2] = logo

#fake velocity function to vaguely simulate flight and test states
def fake_vel(t):
    if t <20:
        return 0
    elif 20 <= t and t < 35:
        return 270-20*(t-20)
    elif 35 <= t and t< 45:
        return -30
    elif 45 <= t and t <55:
        return -5
    else:
        return 0

# --------------------------- MAIN LOOP ---------------------------

def main_lc():
    print('Starting Live Camera GUI...')
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera 0")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    #logo, options are white, blue, or black
    logo = cv2.imread('images/QRET_white.png', cv2.IMREAD_UNCHANGED)
    if logo is not None:
        #resizing logo
        max_logo_width = FRAME_WIDTH//6
        scale = min(1.0, max_logo_width / logo.shape[1])
        logo = cv2.resize(logo, (int(logo.shape[1]*scale), int(logo.shape[0]*scale)), interpolation=cv2.INTER_AREA)
    else:
        print("Logo image not found or failed to load.")

    #time variables 
    t0 = time.time()
    t_check = t0

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    #booleans for state machine
    parachute_deployed = False
    boost_achieved = False

    #video writers
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    raw_writer = cv2.VideoWriter(f'video_output/{now_str}_raw_capture.avi', fourcc, 15.0, (FRAME_WIDTH, FRAME_HEIGHT))
    overlay_writer = cv2.VideoWriter(f'video_output/{now_str}_overlay_capture.avi', fourcc, 15.0, (FRAME_WIDTH, FRAME_HEIGHT))

    #preliminary initialization to avoid error
    data = {
        "mission_time": 0,
        "uptime": 0,
        "altitude": 0,
        "velocity": 0,
        "acceleration": 0,
        "latitude": 0,
        "longitude": 0,
        "pressure": 0,
        "battery": 0,
        "int_temp": 0,
        "video_conn_ok": True,
        "current_stage": 0,
        "gs_connected": True
    }

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            #save raw frame
            raw_writer.write(frame.copy())

            #retrieving data
            prev_data = data
            data = get_live_data(t0, prev_data)

            # TOP STATUS BAR WITH MODERN STYLING ---------------------------
            margin = 15
            current_time = datetime.now().strftime("%H:%M:%S UTC")
            mission_time = f"T+{data['mission_time']:.1f}s"
            
            data['velocity'] = fake_vel(time.time()-t0)            
           
            #check video status and state every 5 seconds
            if time.time() - t_check > 1:
                t_check_conn = time.time()

                #convert to grayscale and measure mean
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                mean_brightness = gray.mean()

                #if image is dark, camera likely isn't connected
                if mean_brightness < 25:
                    data['video_conn_ok'] = False
                else:
                    data['video_conn_ok'] = True


                #non rigerous state machine lol, just velocity based

                ascent_check = data['velocity'] > 5
                drogue_check = data['velocity'] < -25 and data['velocity'] > -45
                main_check = data['velocity'] < 0 and data['velocity'] > -20
                landed_check = data['velocity'] < 5 and data['velocity'] > -5
                ballistic_check = data['velocity'] < -45

                if landed_check and not (parachute_deployed or boost_achieved):#rail
                    status=0
                elif ascent_check:#boost
                    status=1
                    boost_achieved=True
                elif drogue_check:#drogue
                    status=2
                    parachute_deployed=True
                elif main_check:#main
                    status=3
                    parachute_deployed=True
                elif landed_check and parachute_deployed:#landed
                    status=4
                elif ballistic_check and parachute_deployed:#ballistic
                    status=5
                
                data['current_stage'] = status
                
            # Time display with background
            draw_rounded_rect(frame, (margin - 5, 5), 350, 35, COLORS['surface'], alpha=0.85)
            draw_text(frame, current_time, (margin + 5, 30), color=COLORS['ascent'], scale=0.7, thickness=2)
            draw_text(frame, mission_time, (margin + 180, 30), color=COLORS['warning'], scale=0.7, thickness=2)

            # LEFT COLUMN - TELEMETRY BARS --------------------------------
            y_pos = 80
            draw_modern_bar(frame, "ALTITUDE (METRES)", data['altitude'], *ALT_RANGE, (margin, y_pos))
            y_pos += 40
            draw_modern_bar(frame, "VELOCITY (M/S)", data['velocity'], *VEL_RANGE, (margin, y_pos))
            y_pos += 40
            draw_modern_bar(frame, "ACCELERATION (M/S²)", data['acceleration'], *ACC_RANGE, (margin, y_pos))
            
            # TOP RIGHT PANEL - MOVED FURTHER TO CORNER ------------------
            right_margin = 15
            right_x = FRAME_WIDTH - 320 - right_margin
            location_items = [
                f"Latitude:  {data['latitude']:.6f}°",
                f"Longitude: {data['longitude']:.6f}°",
                f"Pressure:  {data['pressure']:.1f} kPa"
            ]
            draw_info_panel(frame, "LOCATION & ENVIRONMENT", location_items, (right_x, 10), width=320)
            
            # Battery bar in top right
            battery_y = 160
            draw_modern_bar(frame, "BATTERY (VOLTS)", data['battery'], *BATT_RANGE, 
                          (right_x + 15, battery_y), length=150)

            # CENTER CROSSHAIR WITH MODERN STYLING ------------------------
            center = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
            draw_crosshair(frame, center)

            # BOTTOM STATUS INDICATORS ------------------------------------
            btm_y = FRAME_HEIGHT - 20
            
            # Kuhglocke connection indicator
            draw_modern_indicator(frame, "KUHGLOCKE", data['gs_connected'], (margin + 150, btm_y))

            # Video connection indicator
            draw_modern_indicator(frame, "VIDEO LINK", data['video_conn_ok'], (margin + 10, btm_y))

            # FLIGHT STAGE PROGRESSION ------------------------------------
            stage_y = FRAME_HEIGHT - 35
            stage_x = FRAME_WIDTH // 2 - 225
            draw_stage_progress(frame, data['current_stage'], (stage_x, stage_y))

            # DRAW LOGO
            draw_logo(frame, logo)

            #save overlaid video
            overlay_writer.write(frame)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break
    finally:
        cap.release()
        raw_writer.release()
        overlay_writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main_lc()