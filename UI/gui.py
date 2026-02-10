import sys
import time
import csv
import threading
import json
import queue
import serial
import serial.tools.list_ports
from collections import deque

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ===================== CONFIGURATION =====================
PORT = "COM7"  # Change if needed
BAUD = 57600
LOG_FILENAME = "temperature_log.csv"
PROFILE_FILE = "reflow_profiles.json"

# Graph Settings
X_SIZE = 300
SMOOTH_WINDOW = 20

# Default "Safe" Parameters if file missing
DEFAULT_PROFILES = {
    "Lead-Free SAC305": {"soak_temp": 150, "soak_time": 90, "reflow_temp": 245, "reflow_time": 60},
    "Leaded Sn63Pb37":  {"soak_temp": 150, "soak_time": 60, "reflow_temp": 183, "reflow_time": 45},
    "Test":             {"soak_temp": 50,  "soak_time": 10, "reflow_temp": 70,  "reflow_time": 10},
}

# ===================== GLOBAL STATE =====================
data_queue = queue.Queue()
stop_event = threading.Event()
ser = None

# Data Buffers
xdata = deque(maxlen=X_SIZE)
yraw = deque(maxlen=X_SIZE)
yfilt = deque(maxlen=X_SIZE)
ytarget = deque(maxlen=X_SIZE) # For the dashed line

current_status = "IDLE"
is_running = False
start_time = 0

# ===================== SERIAL HANDLER =====================
def open_serial():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        return True
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        return False

def send_command(cmd):
    if ser and ser.is_open:
        try:
            full_cmd = f"{cmd}\n"
            ser.write(full_cmd.encode())
            print(f"TX -> {cmd}")
        except Exception as e:
            print(f"TX Error: {e}")

def serial_thread_func():
    """ Runs in background, reads MCU data, puts in queue """
    while not stop_event.is_set():
        if ser and ser.is_open:
            try:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    # Expecting format "TEMP 24.5" or similar
                    parts = line.split()
                    if len(parts) >= 2 and parts[0] == "TEMP":
                        try:
                            val = float(parts[1])
                            data_queue.put(("TEMP", val))
                        except ValueError:
                            pass
            except Exception:
                pass
        time.sleep(0.01)

# ===================== GUI APPLICATION =====================
class ReflowGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Reflow Oven Controller")
        self.root.geometry("1100x700")
        
        # Load Profiles
        self.profiles = self.load_profiles()
        self.current_profile_key = list(self.profiles.keys())[0]

        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # --- MAIN LAYOUT ---
        # Left Panel (Controls) | Right Panel (Graph)
        self.paned = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.left_frame = ttk.Frame(self.paned, width=300)
        self.right_frame = ttk.Frame(self.paned)
        self.paned.add(self.left_frame, weight=1)
        self.paned.add(self.right_frame, weight=4)

        # [FIX] BUILD ORDER SWAPPED: Graph First, Then Controls
        self.build_right_panel() 
        self.build_left_panel()
        
        self.start_animation()

    def build_left_panel(self):
        # --- TITLE ---
        lbl_title = ttk.Label(self.left_frame, text="Reflow Profiles", font=("Arial", 14, "bold"))
        lbl_title.pack(pady=(10, 5), anchor="w", padx=10)

        # --- INITIALIZE VARIABLES FIRST ---
        self.var_soak_temp = tk.DoubleVar()
        self.var_soak_time = tk.IntVar()
        self.var_reflow_temp = tk.DoubleVar()
        self.var_reflow_time = tk.IntVar()

        # --- PROFILE LIST ---
        self.listbox = tk.Listbox(self.left_frame, height=8, font=("Arial", 10), exportselection=False)
        self.listbox.pack(fill=tk.X, padx=10, pady=5)
        self.listbox.bind('<<ListboxSelect>>', self.on_profile_select)
        
        # Populate List
        self.refresh_profile_list()

        # --- PARAMETERS EDITOR ---
        param_frame = ttk.LabelFrame(self.left_frame, text="Parameters", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=10)

        # Inputs Grid
        self.create_input(param_frame, "Soak Temp (°C):", self.var_soak_temp, 0)
        self.create_input(param_frame, "Soak Time (s):", self.var_soak_time, 1)
        self.create_input(param_frame, "Reflow Temp (°C):", self.var_reflow_temp, 2)
        self.create_input(param_frame, "Reflow Time (s):", self.var_reflow_time, 3)

        # Update Graph Preview when numbers change
        for var in [self.var_soak_temp, self.var_soak_time, self.var_reflow_temp, self.var_reflow_time]:
            var.trace_add("write", self.update_preview_trace)

        # --- ACTION BUTTONS (SAVE/NEW/DEL) ---
        btn_frame = ttk.Frame(self.left_frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(btn_frame, text="Save Profile", command=self.save_current_profile).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_frame, text="New", command=self.create_new_profile).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_frame, text="Delete", command=self.delete_profile).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # --- CONTROL SECTION ---
        ctrl_frame = ttk.LabelFrame(self.left_frame, text="Oven Control", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=20)

        self.btn_start = ttk.Button(ctrl_frame, text="START REFLOW", command=self.start_process)
        self.btn_start.pack(fill=tk.X, pady=5)

        self.btn_stop = ttk.Button(ctrl_frame, text="EMERGENCY STOP", command=self.stop_process)
        self.btn_stop.pack(fill=tk.X, pady=5)
        
        self.btn_clear = ttk.Button(ctrl_frame, text="Clear Graph", command=self.clear_graph)
        self.btn_clear.pack(fill=tk.X, pady=5)

        # --- STATUS LABEL ---
        self.lbl_status = ttk.Label(self.left_frame, text="Status: IDLE", font=("Consolas", 12), foreground="gray")
        self.lbl_status.pack(side=tk.BOTTOM, pady=20)

    def create_input(self, parent, label, var, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=5)
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=1, sticky="e", pady=5)

    def build_right_panel(self):
        # Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(5, 5), dpi=100)
        self.ax.set_title("Temperature Curve")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.grid(True, linestyle='--', alpha=0.6)

        # Lines
        self.line_target, = self.ax.plot([], [], 'g--', label='Target')
        self.line_filt, = self.ax.plot([], [], 'b-', linewidth=2, label='Filtered')
        self.line_raw, = self.ax.plot([], [], 'r-', linewidth=0.5, alpha=0.5, label='Raw')
        self.ax.legend(loc='upper left')

        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # ================= LOGIC & EVENTS =================
    def load_profiles(self):
        try:
            with open(PROFILE_FILE, "r") as f:
                data = json.load(f)
                return data.get("profiles", DEFAULT_PROFILES)
        except (FileNotFoundError, json.JSONDecodeError):
            return DEFAULT_PROFILES

    def refresh_profile_list(self):
        self.listbox.delete(0, tk.END)
        for name in self.profiles:
            self.listbox.insert(tk.END, name)
        # Select first one
        if self.listbox.size() > 0:
            self.listbox.selection_set(0)
            self.load_fields_from_selection()

    def on_profile_select(self, event):
        self.load_fields_from_selection()

    def load_fields_from_selection(self):
        try:
            idx = self.listbox.curselection()[0]
            name = self.listbox.get(idx)
            self.current_profile_key = name
            data = self.profiles[name]
            
            # Update vars (disable tracing temporarily to prevent lag?) No need.
            self.var_soak_temp.set(data.get("soak_temp", 0))
            self.var_soak_time.set(data.get("soak_time", 0))
            self.var_reflow_temp.set(data.get("reflow_temp", 0))
            self.var_reflow_time.set(data.get("reflow_time", 0))
            
            self.update_preview()
        except IndexError:
            pass

    def update_preview_trace(self, *args):
        self.update_preview()

    def update_preview(self):
        """ Calculates the Target Curve based on the 4 params """
        try:
            st = self.var_soak_temp.get()
            s_time = self.var_soak_time.get()
            rt = self.var_reflow_temp.get()
            r_time = self.var_reflow_time.get()
            
            # Construct visualization points (Approximate logic)
            # 1. Ramp to Soak (Assume 2C/sec from 25C)
            ramp_up_time = int((st - 25) / 1.5)  
            if ramp_up_time < 1: ramp_up_time = 1
            
            # 2. Soak Phase
            # 3. Ramp to Reflow (Assume 1.5C/sec)
            ramp_reflow_time = int((rt - st) / 1.5)
            if ramp_reflow_time < 1: ramp_reflow_time = 1
            
            # Generate X, Y arrays
            xs = []
            ys = []
            
            # T=0
            xs.append(0); ys.append(25)
            
            # Reach Soak
            xs.append(ramp_up_time); ys.append(st)
            
            # End Soak
            time_end_soak = ramp_up_time + s_time
            xs.append(time_end_soak); ys.append(st)
            
            # Reach Reflow
            time_start_reflow = time_end_soak + ramp_reflow_time
            xs.append(time_start_reflow); ys.append(rt)
            
            # End Reflow
            time_end_reflow = time_start_reflow + r_time
            xs.append(time_end_reflow); ys.append(rt)
            
            # Cooldown (Visual only)
            xs.append(time_end_reflow + 60); ys.append(50)

            self.line_target.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()
            
        except tk.TclError:
            pass # Empty fields
        except AttributeError:
            pass # Graph not ready yet (Safety catch)

    def save_current_profile(self):
        name = self.current_profile_key
        try:
            self.profiles[name] = {
                "soak_temp": self.var_soak_temp.get(),
                "soak_time": self.var_soak_time.get(),
                "reflow_temp": self.var_reflow_temp.get(),
                "reflow_time": self.var_reflow_time.get()
            }
            with open(PROFILE_FILE, "w") as f:
                json.dump({"selected": name, "profiles": self.profiles}, f, indent=2)
            messagebox.showinfo("Saved", f"Profile '{name}' saved.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def create_new_profile(self):
        # Ask for name
        def save_new(name_win, entry):
            new_name = entry.get().strip()
            if new_name:
                self.profiles[new_name] = self.profiles[self.current_profile_key].copy()
                self.refresh_profile_list()
                name_win.destroy()
        
        top = tk.Toplevel(self.root)
        top.title("New Profile")
        ttk.Label(top, text="Profile Name:").pack(padx=10, pady=5)
        ent = ttk.Entry(top)
        ent.pack(padx=10, pady=5)
        ttk.Button(top, text="Create", command=lambda: save_new(top, ent)).pack(pady=5)

    def delete_profile(self):
        if len(self.profiles) <= 1:
            messagebox.showwarning("Stop", "Cannot delete the last profile.")
            return
        if messagebox.askyesno("Confirm", f"Delete '{self.current_profile_key}'?"):
            del self.profiles[self.current_profile_key]
            self.refresh_profile_list()

    # ================= OVEN COMMANDS =================
    def start_process(self):
        global is_running, start_time
        if is_running: return
        
        # 1. Send Parameters to MCU
        # Protocol: SET:PARAM:VALUE
        st = int(self.var_soak_temp.get())
        stime = int(self.var_soak_time.get())
        rt = int(self.var_reflow_temp.get())
        rtime = int(self.var_reflow_time.get())
        
        print("Sending Params to MCU...")
        send_command(f"S:{st}")   # Soak Temp
        time.sleep(0.05)
        send_command(f"K:{stime}") # Soak Time (K to differ from S)
        time.sleep(0.05)
        send_command(f"R:{rt}")   # Reflow Temp
        time.sleep(0.05)
        send_command(f"L:{rtime}") # Reflow Time (L for Length)
        time.sleep(0.05)
        
        # 2. Send Start Command
        send_command("RUN:1")
        
        # 3. Update UI
        is_running = True
        start_time = time.time()
        self.clear_graph()
        self.lbl_status.config(text="Status: RUNNING", foreground="green")

    def stop_process(self):
        global is_running
        send_command("RUN:0")
        is_running = False
        self.lbl_status.config(text="Status: STOPPED", foreground="red")

    def clear_graph(self):
        xdata.clear()
        yraw.clear()
        yfilt.clear()
        self.line_filt.set_data([], [])
        self.line_raw.set_data([], [])
        self.canvas.draw_idle()

    # ================= ANIMATION LOOP =================
    def start_animation(self):
        def animate(i):
            while not data_queue.empty():
                msg, val = data_queue.get()
                if msg == "TEMP":
                    current_t = 0
                    if is_running:
                        current_t = time.time() - start_time
                    else:
                        if len(xdata) > 0: current_t = xdata[-1] + 0.1
                    
                    xdata.append(current_t)
                    yraw.append(val)
                    
                    # Simple moving average filter
                    if len(yraw) >= 5:
                        filt = sum(list(yraw)[-5:]) / 5.0
                    else:
                        filt = val
                    yfilt.append(filt)

            # Update Plot
            self.line_filt.set_data(xdata, yfilt)
            self.line_raw.set_data(xdata, yraw)
            
            if len(xdata) > 0:
                self.ax.set_xlim(0, max(xdata[-1] + 10, 60))
                self.ax.set_ylim(0, max(max(yraw, default=0) + 20, 250))

            return self.line_filt, self.line_raw

        self.anim = animation.FuncAnimation(self.fig, animate, interval=100, blit=False)


# ===================== MAIN =====================
if __name__ == "__main__":
    if open_serial():
        print(f"Connected to {PORT}")
    else:
        print("Running in simulation/offline mode")
        
    # Start Serial Thread
    t = threading.Thread(target=serial_thread_func, daemon=True)
    t.start()

    # Run GUI
    root = tk.Tk()
    app = ReflowGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (stop_event.set(), root.destroy()))
    root.mainloop()