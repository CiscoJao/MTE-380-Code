import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from ball_detection import detect_ball_x, detect_ball_y

class BasicPIDController:
    def __init__(self, config_file="config.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        # PID gains (controlled by sliders in GUI)
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0
        # Scale factor for converting from pixels to meters
        self.scale_factor_x = self.config['calibration']['pixel_to_meter_ratio_x'] * self.config['camera']['frame_width'] / 2
        self.scale_factor_y = self.config['calibration']['pixel_to_meter_ratio_y'] * self.config['camera']['frame_height'] / 2

        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None

        # Controller-internal state
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0

        self.num_axes = 3
        self.integral = np.zeros(self.num_axes)
        self.prev_error = np.zeros(self.num_axes)

        self.Kp_arr = []
        self.Ki_arr = []
        self.Kd_arr = []

        # Data logs for plotting results
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown

        self.M = np.array([
            [1, 0],
            [-0.5, np.sqrt(3)/2],
            [-0.5, -np.sqrt(3)/2]
        ])

    def connect_servo(self): 
        """Try to open serial connection to servo, return True if success.""" 
        try: 
            self.servo = serial.Serial(self.servo_port, 9600) 
            time.sleep(2) 
            print("[SERVO] Connected") 
            return True 
        except Exception as e: 
            print(f"[SERVO] Failed: {e}") 
            return False

    def send_servo_angles(self, angles):
        """Send angle command to servo motor (clipped for safety)."""
        if self.servo:
            temp = []
            for servo_angle in angles:
                servo_angle = self.neutral_angle + servo_angle
                servo_angle = int(np.clip(servo_angle, 0, 30))

                servo_pwm = int((10/3) * servo_angle + 100)
                temp.append(servo_pwm)
            try:
                print("[CONTROL]: ", temp)
                self.servo.write(bytes(temp))
            except Exception:
                print("[SERVO] Send failed")

    def update_pid(self, position, dt):

        pos_vec = np.array([position[0], position[1]])
        set_vec = np.array([self.setpoint_x, self.setpoint_y])
        pos_abc = self.M.dot(pos_vec)
        set_abc = self.M.dot(set_vec)

        print("POSITION AND SETPOINT!!!!!!!!!")
        print("pos_vec = ", pos_vec)
        print("set_abc = ", set_abc)

        outputs = np.zeros(self.num_axes, dtype=float)
        for i in range(self.num_axes):
            error =  pos_abc[i] - set_abc[i] # Compute error

            # error = error * 1  # Scale error for easier tuning (if needed)

            # Proportional term
            P = self.Kp_arr[i] * error

            # Integral term accumulation
            if abs(error) < 0.01:
                self.integral[i] = 0.0
            else:
                self.integral[i] += error * dt

            I = self.Ki_arr[i] * self.integral[i]
            # Derivative term calculation
            derivative = (error - self.prev_error[i]) / dt
            D = self.Kd_arr[i] * derivative
            self.prev_error[i] = error
            # PID output (limit to safe beam range)
            output = P + I + D
            output = np.clip(output, -15, 15)
            outputs[i] = output
        return outputs

    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.resize(frame, (320, 240))
            # Detect ball position in frame
            found_x, x_normalized, vis_frame = detect_ball_x(frame)
            found_y, y_normalized, vis_frame = detect_ball_y(frame)
            if found_x and found_y:
                # Convert normalized to meters using scale
                print("x_normalized = ", x_normalized)
                position_m = (x_normalized * self.scale_factor_x, y_normalized * self.scale_factor_y)

                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait(position_m)
                except Exception:
                    pass
            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        cap.release()
        cv2.destroyAllWindows()

    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")
        self.start_time = time.time()
        last_time = self.start_time
        while self.running:
            try:
                # Wait for latest ball position from camera
                position = self.position_queue.get(timeout=0.1)

                now = time.time()
                dt = now - last_time if last_time is not None else 0.033
                last_time = now
                setpoint_xy = (self.setpoint_x, self.setpoint_y)
                # Compute control output using PID
                control_output = self.update_pid(position, dt)
                # Send control command to servo (real or simulated)

                self.send_servo_angles(control_output)
                # Log results for plotting
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.position_log.append(position)
                self.setpoint_log.append(setpoint_xy)
                self.control_log.append(control_output)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break
        if self.servo:
            # Return to neutral on exit
            self.send_servo_angles([0,0,0])
            self.servo.close()

    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Basic PID Controller")
        self.root.geometry("520x400")

        # Title label
        ttk.Label(self.root, text="PID Gains", font=("Arial", 18, "bold")).pack(pady=10)

        # Kp slider
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 12)).pack()
        self.kp_var = tk.DoubleVar(value=self.Kp)
        kp_slider = ttk.Scale(self.root, from_=0, to=50, variable=self.kp_var,
                              orient=tk.HORIZONTAL, length=500)
        kp_slider.pack(pady=5)
        self.kp_label = ttk.Label(self.root, text=f"Kp: {self.Kp:.1f}", font=("Arial", 11))
        self.kp_label.pack()

        # Ki slider
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 12)).pack()
        self.ki_var = tk.DoubleVar(value=self.Ki)
        ki_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_var,
                              orient=tk.HORIZONTAL, length=500)
        ki_slider.pack(pady=5)
        self.ki_label = ttk.Label(self.root, text=f"Ki: {self.Ki:.1f}", font=("Arial", 11))
        self.ki_label.pack()

        # Kd slider
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 12)).pack()
        self.kd_var = tk.DoubleVar(value=self.Kd)
        kd_slider = ttk.Scale(self.root, from_=0, to=30, variable=self.kd_var,
                              orient=tk.HORIZONTAL, length=500)
        kd_slider.pack(pady=5)
        self.kd_label = ttk.Label(self.root, text=f"Kd: {self.Kd:.1f}", font=("Arial", 11))
        self.kd_label.pack()

        # Setpoint slider
        ttk.Label(self.root, text="Setpoint (meters)", font=("Arial", 12)).pack()
        pos_min = self.config['calibration']['position_x_min_m']
        pos_max = self.config['calibration']['position_x_max_m']
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                   variable=self.setpoint_x_var,
                                   orient=tk.HORIZONTAL, length=500)
        setpoint_slider.pack(pady=5)
        self.setpoint_x_label = ttk.Label(self.root, text=f"Setpoint: {self.setpoint_x:.3f}m", font=("Arial", 11))
        self.setpoint_x_label.pack()

        # Setpoint slider
        ttk.Label(self.root, text="Setpoint (meters)", font=("Arial", 12)).pack()
        pos_min = self.config['calibration']['position_y_min_m']
        pos_max = self.config['calibration']['position_y_max_m']
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                   variable=self.setpoint_y_var,
                                   orient=tk.HORIZONTAL, length=500)
        setpoint_slider.pack(pady=5)
        self.setpoint_y_label = ttk.Label(self.root, text=f"Setpoint: {self.setpoint_y:.3f}m", font=("Arial", 11))
        self.setpoint_y_label.pack()

        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
        ttk.Button(button_frame, text="Reset Integral",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        # Schedule periodic GUI update
        self.update_gui()

    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters
            self.Kp = self.kp_var.get()
            self.Ki = self.ki_var.get()
            self.Kd = self.kd_var.get()

            self.Kp_arr = np.ones(self.num_axes) * self.Kp
            self.Ki_arr = np.ones(self.num_axes) * self.Ki
            self.Kd_arr = np.ones(self.num_axes) * self.Kd

            self.setpoint_x = self.setpoint_x_var.get()
            self.setpoint_y = self.setpoint_y_var.get()

            # Update displayed values
            self.kp_label.config(text=f"Kp: {self.Kp:.1f}")
            self.ki_label.config(text=f"Ki: {self.Ki:.1f}")
            self.kd_label.config(text=f"Kd: {self.Kd:.1f}")

            self.setpoint_x_label.config(text=f"Setpoint X: {self.setpoint_x:.3f}m")
            self.setpoint_y_label.config(text=f"Setpoint Y: {self.setpoint_y:.3f}m")

            # Call again after 50 ms (if not stopped)
            self.root.after(50, self.update_gui)

    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral = np.zeros(self.num_axes, dtype=float)
        print("[RESET] Integral term reset")

    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig, (ax3, ax4, ax5) = plt.subplots(3, 1, figsize=(10, 8))

        position_log_x = [pos[0] for pos in self.position_log]
        position_log_y = [pos[1] for pos in self.position_log]

        setpoint_log_x = [sp[0] for sp in self.setpoint_log]
        setpoint_log_y = [sp[1] for sp in self.setpoint_log]

        control_output_a = [ctrl[0] for ctrl in self.control_log]
        control_output_b = [ctrl[1] for ctrl in self.control_log]
        control_output_c = [ctrl[2] for ctrl in self.control_log]
        
        # Ball position X trace
        ax1.plot(self.time_log, position_log_x, label="Ball Position", linewidth=2)
        ax1.plot(self.time_log, setpoint_log_x, label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("X Position (m)")
        ax1.set_title(f"Basic PID Control (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Ball position Y trace
        ax2.plot(self.time_log, position_log_y, label="Ball Position", linewidth=2)
        ax2.plot(self.time_log, setpoint_log_y, label="Setpoint",
                 linestyle="--", linewidth=2)
        ax2.set_ylabel("Y Position (m)")
        ax2.set_title(f"Basic PID Control (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Control output trace
        ax3.plot(self.time_log, control_output_a, label="Control Output A",
                 color="orange", linewidth=2)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Motor A Angle (degrees)")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        ax4.plot(self.time_log, control_output_b, label="Control Output B",
                 color="green", linewidth=2)
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Motor B Angle (degrees)")
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        ax5.plot(self.time_log, control_output_c, label="Control Output C",
            color="red", linewidth=2)
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("Motor C Angle (degrees)")
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False
        # Try to safely close all windows/resources
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()

        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_autocal.py first.")
    except Exception as e:
        print(f"[ERROR] {e}")
