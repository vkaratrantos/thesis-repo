import tkinter as tk
from tkinter import ttk, messagebox
import datetime
import threading
import time
import sys

# --- ROS 2 LIBRARIES ---
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

# Configuration
REAGENTS = {
    "Tube 1": "Copper Sulfate (CuSO4)",
    "Tube 2": "Sodium Hydroxide (NaOH)",
    "Tube 3": "Sodium Carbonate (Na2CO3)",
    "Tube 4": "Hydrochloric Acid (HCl)",
    "Tube 5": "Ammonia (NH3)"
}

RECIPES = {
    "Copper Hydroxide - Cu(OH)2": [1, 2],
    "Copper Carbonate - CuCO3": [1, 3],
    "Carbon Dioxide - CO2": [3, 4],
    "Ammonium Chloride - NH4Cl": [4, 5],
    "Royal Blue Complex - [Cu(NH3)4]2+": [1, 5],
    "Neutralization": [4, 2]
}

# Colors
COLOR_BG = "#0a0a0a"
COLOR_PANEL = "#141414"
COLOR_BLUE_NEON = "#0066cc"
COLOR_BLUE_DARK = "#004488"
COLOR_TEXT_MAIN = "#ffffff"
COLOR_TEXT_DIM = "#888888"
COLOR_BORDER = "#ffffff"  

class LabApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Automated Chemical Synthesis")
        self.root.geometry("1200x850") 
        self.root.configure(bg=COLOR_BG)

        self.manual_batch = [] 
        
        # FIX: Safety locks and debouncers to prevent double-firing
        self.sequence_lock = threading.Lock()
        self.last_manual_cmd = ""
        self.last_manual_time = 0

        # --- NATIVE ROS 2 INITIALIZATION ---
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('gui_commander_node')
        self.publisher = self.ros_node.create_publisher(String, '/gui_commands', 10)

        # Outcome of each command, so a sequence can stop when a step fails.
        # simple_move publishes "OK <cmd>" or "FAIL <cmd>" on /gui_status.
        self.status_lock = threading.Lock()
        self.status_event = threading.Event()
        self.pending_cmd = None
        self.last_result = None
        self.abort_requested = False
        self.ros_node.create_subscription(String, '/gui_status', self.status_cb, 10)

        # The node has to be spun to RECEIVE anything. Publishing worked without
        # this, which is why it was never needed before.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # Ensure ROS 2 shuts down cleanly when you close the window
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Header
        header_frame = tk.Frame(root, bg=COLOR_BG, pady=20)
        header_frame.pack(fill="x")
        
        tk.Label(header_frame, text="CHEMICAL SYNTHESIS", font=("Arial", 36, "bold"), bg=COLOR_BG, fg=COLOR_BLUE_NEON).pack()
        tk.Label(header_frame, text="AUTOMATED FLUID HANDLING SYSTEM", font=("Arial", 16), bg=COLOR_BG, fg=COLOR_TEXT_DIM).pack(pady=(0, 10))
        tk.Frame(root, bg=COLOR_BORDER, height=2).pack(fill="x", padx=40, pady=(0, 20))

        # Tabs
        style = ttk.Style()
        style.theme_use('alt') 
        style.configure("TNotebook", background=COLOR_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#222", foreground="#aaa", padding=[10, 15], font=("Arial", 16, "bold"), width=35, anchor="center")
        style.map("TNotebook.Tab", background=[("selected", COLOR_BLUE_NEON)], foreground=[("selected", "white")])

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(expand=False, fill="both", padx=40, pady=10)
        
        self.tab_auto_container = tk.Frame(self.notebook, bg=COLOR_BORDER) 
        self.tab_manual_container = tk.Frame(self.notebook, bg=COLOR_BORDER) 
        self.tab_auto = tk.Frame(self.tab_auto_container, bg=COLOR_PANEL)
        self.tab_auto.pack(fill="both", expand=True, padx=2, pady=2) 
        self.tab_manual = tk.Frame(self.tab_manual_container, bg=COLOR_PANEL)
        self.tab_manual.pack(fill="both", expand=True, padx=2, pady=2) 
        
        self.notebook.add(self.tab_auto_container, text="AUTO SYNTHESIS")
        self.notebook.add(self.tab_manual_container, text="MANUAL CONTROL")
        
        self.build_auto_tab()
        self.build_manual_tab()

        self.log("System Ready. Connected directly to ROS 2 Network.")

    def on_close(self):
        """Safely shuts down the ROS 2 node when the GUI is closed."""
        self.log("Shutting down GUI Node...")
        # Release any sequence blocked in send_and_wait, or the daemon thread
        # sits on its timeout while the window is already gone.
        self.abort_requested = True
        self.status_event.set()
        try:
            self.executor.shutdown()
            # Let spin() actually return before tearing the node down, or rclpy
            # aborts with "terminate called without an active exception".
            self.spin_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()

    def log(self, message):
        """Logs high-level Python events to your terminal."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [GUI] {message}")

    # --- THE MAGIC PIPELINE ---
    def send_command(self, cmd, manual=False):
        """Publishes the command instantly as a native ROS 2 node."""
        
        # FIX: Mouse-Bounce Debouncer for Manual Clicks
        if manual:
            current_time = time.time()
            if cmd == self.last_manual_cmd and (current_time - self.last_manual_time) < 0.3:
                return # Ignore accidental physical double-clicks within 0.3 seconds
            self.last_manual_cmd = cmd
            self.last_manual_time = current_time

        msg = String()
        msg.data = str(cmd)
        self.publisher.publish(msg)
        self.log(f"Command fired -> {cmd}")

    def status_cb(self, msg):
        """Receives 'OK <cmd>' / 'FAIL <cmd>' from simple_move."""
        data = msg.data.strip()
        ok = data.startswith("OK ")
        cmd = data.split(" ", 1)[1].strip() if " " in data else ""
        with self.status_lock:
            # Ignore anything we are not currently waiting on -- a late reply to
            # an aborted step must not release the next wait.
            if self.pending_cmd is None or cmd != self.pending_cmd:
                return
            self.last_result = ok
            self.pending_cmd = None
        self.status_event.set()

    def send_and_wait(self, cmd, timeout):
        """Sends one command and blocks until simple_move reports its outcome.

        Returns False on failure OR timeout. This is what makes a sequence stop
        instead of firing the next step into a robot that never arrived.
        """
        with self.status_lock:
            self.pending_cmd = cmd
            self.last_result = None
        self.status_event.clear()

        self.send_command(cmd)

        if not self.status_event.wait(timeout):
            with self.status_lock:
                self.pending_cmd = None
            self.log(f"    TIMEOUT after {timeout:.0f}s waiting for '{cmd}'")
            return False
        return bool(self.last_result)

    def _run_tube(self, marker_id):
        """One full dispense cycle for a tube, aborting at the first failure.

        Uses the manual command path (m<N>/c/p/o) rather than 'TASK <N>'. The
        MTC pipeline behind TASK does a hard TF lookup and fails outright
        without the camera; this path falls back to the predefined tube
        positions, and it is the path that actually gets exercised and fixed.
        """
        steps = [
            (f"m{marker_id}", 180.0, f"move above tube {marker_id}"),
            ("c",              90.0, "close gripper"),
            ("m6",            180.0, "carry to mixer"),
            ("p",              90.0, "pour"),
            (f"m{marker_id}", 180.0, f"return tube {marker_id}"),
            ("o",              90.0, "release"),
        ]
        for cmd, timeout, label in steps:
            if self.abort_requested:
                self.log("Sequence aborted by user.")
                return False
            self.log(f"  -> {label}  [{cmd}]")
            if not self.send_and_wait(cmd, timeout):
                self.log(f"  !! STEP FAILED: {label}  [{cmd}] -- stopping sequence")
                return False
        return True

    # --- AUTO MODE ---
    def build_auto_tab(self):
        container = tk.Frame(self.tab_auto, bg=COLOR_PANEL)
        container.pack(expand=True, fill="both", pady=40)

        for recipe_name in RECIPES:
            btn = tk.Button(container, text=recipe_name, width=50, height=2, 
                            bg="#222", fg="white", font=("Arial", 16),
                            activebackground=COLOR_BLUE_NEON, activeforeground="black", 
                            relief="flat", bd=0,
                            command=lambda r=recipe_name: self.run_recipe(r))
            btn.pack(pady=10)

    def run_recipe(self, recipe_name):
        markers = RECIPES[recipe_name]
        self.log(f"Auto-Sequence Started: {recipe_name}")

        def task():
            if not self.sequence_lock.acquire(blocking=False):
                self.log("A sequence is already running; ignoring.")
                return
            try:
                self.abort_requested = False
                for i, marker in enumerate(markers, 1):
                    self.log(f"[{i}/{len(markers)}] Tube {marker}")
                    if not self._run_tube(marker):
                        self.log(f"Auto-Sequence ABORTED at tube {marker}.")
                        return
                self.log(f"Auto-Sequence complete: {recipe_name}")
            finally:
                self.sequence_lock.release()

        threading.Thread(target=task, daemon=True).start()

    # --- MANUAL MODE ---
    def build_manual_tab(self):
        main_frame = tk.Frame(self.tab_manual, bg=COLOR_PANEL)
        main_frame.pack(fill="both", expand=True, padx=40, pady=20)

        # DIRECT OVERRIDE PANEL
        override_frame = tk.LabelFrame(main_frame, text=" DIRECT ROBOT OVERRIDE ", 
                                       bg=COLOR_PANEL, fg=COLOR_BLUE_NEON, font=("Arial", 14, "bold"), pady=15, padx=15)
        override_frame.pack(fill="x", pady=(0, 20))

        actions_frame = tk.Frame(override_frame, bg=COLOR_PANEL)
        actions_frame.pack(pady=5)

        # Note: Added 'manual=True' to engage the debouncer
        tk.Button(actions_frame, text="OPEN GRIPPER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("o", manual=True)).grid(row=0, column=0, padx=10)
        tk.Button(actions_frame, text="CLOSE GRIPPER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("c", manual=True)).grid(row=0, column=1, padx=10)
        tk.Button(actions_frame, text="POUR LIQUID", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("p", manual=True)).grid(row=0, column=2, padx=10)
        tk.Button(actions_frame, text="GO TO MIXER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("m 6", manual=True)).grid(row=0, column=3, padx=10)

        targets_frame = tk.Frame(override_frame, bg=COLOR_PANEL)
        targets_frame.pack(pady=10)

        for i in range(1, 6):
            tk.Button(targets_frame, text=f"GO TO TUBE {i}", bg="#222", fg="white", width=12, font=("Arial", 10), 
                      command=lambda m=i: self.send_command(f"m {m}", manual=True)).grid(row=0, column=i-1, padx=5)

        # MANUAL BATCH
        batch_frame = tk.LabelFrame(main_frame, text=" MANUAL BATCH PROTOCOL ", 
                                    bg=COLOR_PANEL, fg=COLOR_TEXT_DIM, font=("Arial", 14, "bold"), pady=15, padx=15)
        batch_frame.pack(fill="both", expand=True)

        controls_frame = tk.Frame(batch_frame, bg=COLOR_PANEL)
        controls_frame.pack()

        for i, (pump_id, chem_name) in enumerate(REAGENTS.items()):
            tk.Label(controls_frame, text=f"{pump_id} | {chem_name}", width=30, anchor="w", 
                     bg=COLOR_PANEL, fg="#ccc", font=("Arial", 14)).grid(row=i, column=0, padx=15, pady=10)

            btn_add = tk.Button(controls_frame, text="ADD TO SEQUENCE", bg="#333", fg="white", width=18, height=1, relief="flat",
                                font=("Arial", 12, "bold"), 
                                activebackground=COLOR_BLUE_NEON, activeforeground="black",
                                command=lambda p=pump_id, n=chem_name: self.add_to_batch(p, n))
            btn_add.grid(row=i, column=1, padx=15)

        self.batch_lbl = tk.Label(batch_frame, text="SEQUENCE: 0 ITEMS PENDING", font=("Arial", 16, "bold"), bg=COLOR_PANEL, fg=COLOR_TEXT_DIM)
        self.batch_lbl.pack(pady=20)

        btn_exec = tk.Button(batch_frame, text="EXECUTE SEQUENCE", bg=COLOR_BLUE_DARK, fg="white", 
                             font=("Arial", 18, "bold"), width=35, height=2, relief="flat",
                             activebackground=COLOR_BLUE_NEON, activeforeground="black",
                             command=self.execute_batch)
        btn_exec.pack(side="bottom", pady=10)

    def add_to_batch(self, pump_id, name):
        self.manual_batch.append(pump_id)
        self.log(f"Manual Added: {name}")
        self.batch_lbl.config(text=f"SEQUENCE: {len(self.manual_batch)} ITEM(S) PENDING", fg="#00aaff")

    def execute_batch(self):
        if not self.manual_batch:
            messagebox.showwarning("Error", "No ingredients added.")
            return

        self.log("Starting Manual Dispense Protocol...")
        
        # Copy the list and clear the UI immediately
        batch_to_run = list(self.manual_batch)
        self.manual_batch.clear()
        self.batch_lbl.config(text="SEQUENCE: 0 ITEMS PENDING", fg=COLOR_TEXT_DIM)

        def task():
            if not self.sequence_lock.acquire(blocking=False):
                self.log("A sequence is already running; ignoring.")
                return
            try:
                self.abort_requested = False
                for i, tube_string in enumerate(batch_to_run, 1):
                    marker_id = tube_string.split(" ")[1]
                    self.log(f"[{i}/{len(batch_to_run)}] {tube_string}")
                    if not self._run_tube(marker_id):
                        self.log(f"Manual Sequence ABORTED at {tube_string}.")
                        return
                self.log("Manual Sequence Completed.")
            finally:
                self.sequence_lock.release()

        threading.Thread(target=task, daemon=True).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = LabApp(root)
    root.mainloop()
