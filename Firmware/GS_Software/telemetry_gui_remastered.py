import tkinter as tk
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from fetch_transmitter import (
    fetch_transmitter_data,
    calculate_vertical_velocity,
    calculate_vertical_acceleration
)

class TelemetryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Telemetry Data")
        self.root.geometry("800x900")
        self.root.configure(bg="#1e1e1e")

        self.start_time = time.time()
        self.previous_altitude = None
        self.previous_velocity = None
        self.previous_uptime = None
        self.previous_packet_time = None

        # Graph Data
        self.max_points = 30
        self.time_data = deque(maxlen=self.max_points)
        self.altitude_data = deque(maxlen=self.max_points)
        self.velocity_data = deque(maxlen=self.max_points)
        self.acceleration_data = deque(maxlen=self.max_points)

        # ====================
        # UI Layout
        # ====================
        self.label_frame = tk.Frame(root, bg="#1e1e1e")
        self.label_frame.pack(pady=10)

        label_kwargs = {"bg": "#1e1e1e", "fg": "white", "font": ("Segoe UI", 10, "bold"), "anchor": "w"}

        self.data_age_label = tk.Label(self.label_frame, text="Data Age:", **label_kwargs)
        self.data_age_label.grid(row=0, column=0, sticky="w", pady=2)

        self.latitude_label = tk.Label(self.label_frame, text="GPS Latitude:", **label_kwargs)
        self.latitude_label.grid(row=1, column=0, sticky="w", pady=2)

        self.longitude_label = tk.Label(self.label_frame, text="GPS Longitude:", **label_kwargs)
        self.longitude_label.grid(row=2, column=0, sticky="w", pady=2)

        self.satellite_label = tk.Label(self.label_frame, text="GPS Satellite Count:", **label_kwargs)
        self.satellite_label.grid(row=3, column=0, sticky="w", pady=2)

        self.altitude_label = tk.Label(self.label_frame, text="Altitude:", **label_kwargs)
        self.altitude_label.grid(row=4, column=0, sticky="w", pady=2)

        self.velocity_label = tk.Label(self.label_frame, text="Velocity:", **label_kwargs)
        self.velocity_label.grid(row=5, column=0, sticky="w", pady=2)

        self.acceleration_label = tk.Label(self.label_frame, text="Acceleration:", **label_kwargs)
        self.acceleration_label.grid(row=6, column=0, sticky="w", pady=2)

        # ====================
        # Matplotlib Figures
        # ====================
        plt.style.use("dark_background")

        self.fig_alt, self.ax_alt = plt.subplots()
        self.fig_alt.set_size_inches(6, 2)

        self.fig_vel, self.ax_vel = plt.subplots()
        self.fig_vel.set_size_inches(6, 2)

        self.fig_acc, self.ax_acc = plt.subplots()
        self.fig_acc.set_size_inches(6, 2)

        self.canvas_alt = FigureCanvasTkAgg(self.fig_alt, master=root)
        self.canvas_vel = FigureCanvasTkAgg(self.fig_vel, master=root)
        self.canvas_acc = FigureCanvasTkAgg(self.fig_acc, master=root)

        self.ax_alt.set_title("Altitude (ft)", color="white")
        self.ax_vel.set_title("Vertical Velocity (m/s)", color="white")
        self.ax_acc.set_title("Vertical Acceleration (m/s²)", color="white")

        self.alt_line, = self.ax_alt.plot([], [], color="#ffcc00")
        self.vel_line, = self.ax_vel.plot([], [], color="#00ccff")
        self.acc_line, = self.ax_acc.plot([], [], color="#ff4444")

        for ax in [self.ax_alt, self.ax_vel, self.ax_acc]:
            ax.tick_params(axis='x', colors='white')
            ax.tick_params(axis='y', colors='white')
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['left'].set_color('white')
            ax.spines['right'].set_color('white')

        self.canvas_alt.get_tk_widget().pack(pady=5)
        self.canvas_vel.get_tk_widget().pack(pady=5)
        self.canvas_acc.get_tk_widget().pack(pady=5)

        # Start update loop
        self.root.after(1000, self.update_gui)

    def update_gui(self):
        try:
            data = fetch_transmitter_data()
            current_time = time.time()

            if self.previous_packet_time is not None:
                age = current_time - self.previous_packet_time
                self.data_age_label.config(text=f"Data Age: {age:.3f}s")
            self.previous_packet_time = current_time

            # Update labels
            self.latitude_label.config(text=f"GPS Latitude: {data['Lat']}")
            self.longitude_label.config(text=f"GPS Longitude: {data['Lon']}")
            self.satellite_label.config(text=f"GPS Satellite Count: {data['NumSat']}")
            self.altitude_label.config(text=f"Altitude: {data['Alt']} ft")

            # Calculations
            timestamp = time.time() - self.start_time
            velocity = calculate_vertical_velocity(data, self.previous_altitude, self.previous_uptime)
            acceleration = calculate_vertical_acceleration(velocity, data, self.previous_velocity, self.previous_uptime)

            # Save values
            self.time_data.append(timestamp)
            self.altitude_data.append(float(data["Alt"]))
            self.velocity_data.append(velocity)
            self.acceleration_data.append(acceleration)

            # Update previous values
            self.previous_altitude = float(data["Alt"])
            self.previous_uptime = float(data["Uptime"]) / 1000
            self.previous_velocity = velocity

            # Update Graphs
            self.alt_line.set_data(self.time_data, self.altitude_data)
            self.ax_alt.relim()
            self.ax_alt.autoscale_view()
            self.canvas_alt.draw()

            self.vel_line.set_data(self.time_data, self.velocity_data)
            self.ax_vel.relim()
            self.ax_vel.autoscale_view()
            self.canvas_vel.draw()

            self.acc_line.set_data(self.time_data, self.acceleration_data)
            self.ax_acc.relim()
            self.ax_acc.autoscale_view()
            self.canvas_acc.draw()

            # Update calculation labels
            self.velocity_label.config(text=f"Velocity: {velocity:.2f} m/s")
            self.acceleration_label.config(text=f"Acceleration: {acceleration:.2f} m/s²")

        except Exception as e:
            print(f"Error updating GUI: {e}")

        # Call this again after 1000 ms
        self.root.after(1000, self.update_gui)

if __name__ == "__main__":
    print("Starting Telemetry GUI...")
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
