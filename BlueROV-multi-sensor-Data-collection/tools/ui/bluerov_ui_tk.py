#!/usr/bin/env python3
"""
Shahid Ahamed Hasib
UTLN, MSMIR, LIS Lab

"""
import os
import signal
import subprocess
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog


# config

WORKSPACE_DIR = os.path.expanduser("~/bluerov/bluerov_ws")
ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = os.path.join(WORKSPACE_DIR, "install/setup.bash")

COMMANDS = {
    "MAVROS (run_mavros.launch)": "ros2 launch autonomous_rov run_mavros.launch",
    "Listener (run_listener.launch)": "ros2 launch autonomous_rov run_listener.launch",
    "Gamepad (run_gamepad.launch)": "ros2 launch autonomous_rov run_gamepad.launch",
    "Multivideo (3 cams)": "ros2 run autonomous_rov multivideo",
    "Ping360 sonar": "ros2 run ping360_sonar ping360.py",
    "Synced node (cams + sonar)": "ros2 run autonomous_rov synced_node",
}

# Types we consider "video topics"
IMAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}

# Known sonar-related types/topics
SONAR_HINTS = {
    "/scan",
    "/scan_echo",
    "/scan_image",
}
PING1D_TOPIC = "/bluerov2/ping1d/data"


# Helpers

def bash_command(cmd: str) -> str:
    """Run cmd inside bash after sourcing ROS + workspace."""
    if not os.path.exists(ROS_SETUP):
        raise FileNotFoundError(f"ROS setup not found: {ROS_SETUP}")
    if not os.path.exists(WS_SETUP):
        raise FileNotFoundError(f"Workspace setup not found: {WS_SETUP} (did you colcon build?)")

    # Reduce FastDDS SHM noise
    env = "export FASTDDS_SHM_TRANSPORT_DISABLED=1;"
    return f"bash -lc '{env} source {ROS_SETUP} && source {WS_SETUP} && {cmd}'"


def run_cmd_capture(cmd: str, timeout: int = 6) -> str:
    """Run a ROS command and capture stdout (with timeout)."""
    full = bash_command(cmd)
    p = subprocess.run(
        full,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        cwd=WORKSPACE_DIR,
    )
    return p.stdout or ""


def parse_topic_list_with_types(text: str) -> dict:
    """
    Parse output of: ros2 topic list -t
    Expected lines like:
      /topic_name [sensor_msgs/msg/Image]
    Returns: {topic: type}
    """
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if "[" in line and "]" in line:
            # Split "/name [type]"
            try:
                name, rest = line.split("[", 1)
                name = name.strip()
                typ = rest.split("]", 1)[0].strip()
                out[name] = typ
            except Exception:
                continue
    return out


def unique_preserve_order(items):
    seen = set()
    res = []
    for x in items:
        if x not in seen:
            res.append(x)
            seen.add(x)
    return res


class ProcessHandle:
    def __init__(self, name: str, popen: subprocess.Popen):
        self.name = name
        self.popen = popen



# Bag -> Video conversion

def validate_bag_folder(bag_path: str):
    meta = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(meta):
        raise FileNotFoundError(
            f"metadata.yaml not found in:\n{bag_path}\n\n"
            f"Select the bag folder that contains metadata.yaml."
        )


def detect_storage_id(bag_path: str) -> str:
    files = os.listdir(bag_path)
    if any(f.endswith(".mcap") for f in files):
        return "mcap"
    if any(f.endswith(".db3") for f in files):
        return "sqlite3"
    return "sqlite3"


def list_bag_topics_python(bag_path: str, storage_id: str = "auto") -> dict:
    """
    Returns dict {topic: type} from a bag using rosbag2_py.
    """
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    validate_bag_folder(bag_path)
    if storage_id in ("auto", "", None):
        storage_id = detect_storage_id(bag_path)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    return {t.name: t.type for t in topic_types}


def convert_bag_to_video(bag_path: str, storage_id: str, topic_name: str,
                         output_path: str, fps: int, log_cb):
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rosidl_runtime_py.utilities import get_message

    import cv2
    import numpy as np

    try:
        from cv_bridge import CvBridge
        bridge = CvBridge()
    except Exception:
        bridge = None

    validate_bag_folder(bag_path)

    if storage_id in ("auto", "", None):
        storage_id = detect_storage_id(bag_path)

    log_cb(f"[INFO] Bag: {bag_path}\n")
    log_cb(f"[INFO] Storage: {storage_id}\n")

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    topic_type_dict = {t.name: t.type for t in topic_types}
    if not topic_type_dict:
        raise RuntimeError("No topics found in bag (maybe 0 messages).")

    if topic_name not in topic_type_dict:
        raise ValueError(
            f"Topic '{topic_name}' not found in bag.\nAvailable:\n" +
            "\n".join(topic_type_dict.keys())
        )

    msg_type_str = topic_type_dict[topic_name]
    log_cb(f"[INFO] Topic type: {msg_type_str}\n")

    reader.set_filter(StorageFilter(topics=[topic_name]))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = None
    frame_size = None
    frame_count = 0

    MsgClass = get_message(msg_type_str)

    rclpy.init()
    log_cb("[INFO] Converting...\n")

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, MsgClass)

        if msg_type_str.endswith("sensor_msgs/msg/Image"):
            if bridge is None:
                raise RuntimeError("cv_bridge not available, cannot decode sensor_msgs/Image")
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                log_cb(f"[WARN] skip frame: {e}\n")
                continue

        elif msg_type_str.endswith("sensor_msgs/msg/CompressedImage"):
            try:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    raise RuntimeError("cv2.imdecode returned None")
            except Exception as e:
                log_cb(f"[WARN] skip frame: {e}\n")
                continue
        else:
            raise ValueError(f"Unsupported type for video conversion: {msg_type_str}")

        if frame_size is None:
            frame_size = (cv_image.shape[1], cv_image.shape[0])
            os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
            video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)

        video_writer.write(cv_image)
        frame_count += 1

    if video_writer:
        video_writer.release()
        log_cb(f"[OK] Saved: {output_path}  frames={frame_count}\n")
    else:
        log_cb("[WARN] No frames written.\n")

    rclpy.shutdown()



# UI

class BlueROVUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("BlueROV Control Panel")
        self.geometry("1220x900")

        self.proc_handles: dict[str, ProcessHandle] = {}
        self.record_handle: ProcessHandle | None = None
        self.log_queue = queue.Queue()

        self.vars = {k: tk.BooleanVar(value=False) for k in COMMANDS.keys()}

        # Recording vars
        self.bag_out_var = tk.StringVar(value=os.path.expanduser("~/rosbags/session_01"))
        self.storage_var = tk.StringVar(value="mcap")

        # Conversion vars
        self.conv_bag_var = tk.StringVar(value=os.path.expanduser("~/rosbags/session_01"))
        self.conv_storage_var = tk.StringVar(value="auto")
        self.conv_topic_var = tk.StringVar(value="")
        self.conv_out_var = tk.StringVar(value=os.path.expanduser("~/rosbags/output.mp4"))
        self.conv_fps_var = tk.IntVar(value=30)

        self.detected_topics = {}  # {topic: type}

        self._build_ui()
        self._start_log_pump()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self._log("UI ready. Start your nodes, then click Detect All.\n")

    #  UI 
    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text=f"Workspace: {WORKSPACE_DIR}").grid(row=0, column=0, sticky="w")
        ttk.Label(top, text=f"ROS: {ROS_SETUP}").grid(row=1, column=0, sticky="w")

        btns = ttk.Frame(top)
        btns.grid(row=0, column=1, rowspan=2, sticky="e")
        ttk.Button(btns, text="Start Selected", command=self.start_selected).grid(row=0, column=0, padx=5)
        ttk.Button(btns, text="Stop All", command=self.stop_all).grid(row=0, column=1, padx=5)
        ttk.Button(btns, text="Status", command=self.show_status).grid(row=0, column=2, padx=5)

        top.columnconfigure(0, weight=1)

        mod = ttk.LabelFrame(self, text="Modules", padding=10)
        mod.pack(fill="x", padx=10, pady=5)

        r = 0
        for name in COMMANDS.keys():
            ttk.Checkbutton(mod, text=name, variable=self.vars[name]).grid(
                row=r // 2, column=r % 2, sticky="w", padx=10, pady=5
            )
            r += 1
        mod.columnconfigure(0, weight=1)
        mod.columnconfigure(1, weight=1)

        rec = ttk.LabelFrame(self, text="Recording (auto-detect topics)", padding=10)
        rec.pack(fill="x", padx=10, pady=5)

        ttk.Label(rec, text="Bag output (-o):").grid(row=0, column=0, sticky="w")
        ttk.Entry(rec, textvariable=self.bag_out_var, width=70).grid(row=0, column=1, sticky="we", padx=5)
        ttk.Button(rec, text="Browse", command=self.browse_bag_out).grid(row=0, column=2, padx=5)

        ttk.Label(rec, text="Storage:").grid(row=1, column=0, sticky="w")
        ttk.Combobox(rec, textvariable=self.storage_var, values=["mcap", "sqlite3"],
                     width=10, state="readonly").grid(row=1, column=1, sticky="w", padx=5)

        # Detect buttons
        detect_frame = ttk.Frame(rec)
        detect_frame.grid(row=1, column=2, sticky="e")
        ttk.Button(detect_frame, text="Detect Cameras", command=self.detect_cameras).grid(row=0, column=0, padx=3)
        ttk.Button(detect_frame, text="Detect Sonar", command=self.detect_sonar).grid(row=0, column=1, padx=3)
        ttk.Button(detect_frame, text="Detect All", command=self.detect_all).grid(row=0, column=2, padx=3)

        ttk.Label(rec, text="Topics to record (one per line):").grid(row=2, column=0, sticky="nw", pady=(8, 0))
        self.topics_box = tk.Text(rec, height=7, width=70)
        self.topics_box.grid(row=2, column=1, sticky="we", padx=5, pady=(8, 0))

        rec_btns = ttk.Frame(rec)
        rec_btns.grid(row=2, column=2, sticky="n", pady=(8, 0))
        ttk.Button(rec_btns, text="Start Recording", command=self.start_recording).grid(row=0, column=0, pady=2, sticky="ew")
        ttk.Button(rec_btns, text="Stop Recording", command=self.stop_recording).grid(row=1, column=0, pady=2, sticky="ew")
        ttk.Button(rec_btns, text="List Live Topics", command=self.refresh_detected_topics).grid(row=2, column=0, pady=2, sticky="ew")

        rec.columnconfigure(1, weight=1)

        conv = ttk.LabelFrame(self, text="Bag → Video", padding=10)
        conv.pack(fill="x", padx=10, pady=5)

        ttk.Label(conv, text="Bag folder:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conv, textvariable=self.conv_bag_var, width=70).grid(row=0, column=1, sticky="we", padx=5)
        ttk.Button(conv, text="Browse", command=self.browse_conv_bag).grid(row=0, column=2, padx=5)

        ttk.Label(conv, text="Storage:").grid(row=1, column=0, sticky="w")
        ttk.Combobox(conv, textvariable=self.conv_storage_var, values=["auto", "mcap", "sqlite3"],
                     width=10, state="readonly").grid(row=1, column=1, sticky="w", padx=5)

        ttk.Button(conv, text="Load Bag Topics", command=self.load_bag_topics_to_dropdown).grid(row=1, column=2, padx=5)

        ttk.Label(conv, text="Video topic:").grid(row=2, column=0, sticky="w")
        self.topic_dropdown = ttk.Combobox(conv, textvariable=self.conv_topic_var, values=[], width=62)
        self.topic_dropdown.grid(row=2, column=1, sticky="w", padx=5)

        ttk.Label(conv, text="Output mp4:").grid(row=3, column=0, sticky="w")
        ttk.Entry(conv, textvariable=self.conv_out_var, width=70).grid(row=3, column=1, sticky="we", padx=5)
        ttk.Button(conv, text="Browse", command=self.browse_conv_out).grid(row=3, column=2, padx=5)

        ttk.Label(conv, text="FPS:").grid(row=4, column=0, sticky="w")
        ttk.Spinbox(conv, from_=1, to=120, textvariable=self.conv_fps_var, width=8).grid(row=4, column=1, sticky="w", padx=5)

        ttk.Button(conv, text="Convert Now", command=self.start_conversion).grid(row=4, column=2, padx=5)

        conv.columnconfigure(1, weight=1)

        logs = ttk.LabelFrame(self, text="Logs", padding=10)
        logs.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(logs, wrap="word", height=18)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(logs, orient="vertical", command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    #  logging 
    def _log(self, msg: str):
        self.log_queue.put(msg)

    def _start_log_pump(self):
        def pump():
            try:
                while True:
                    msg = self.log_queue.get_nowait()
                    self.log_text.insert("end", msg)
                    self.log_text.see("end")
            except queue.Empty:
                pass
            self.after(100, pump)
        pump()

    # processes 
    def _reader_thread(self, name: str, popen: subprocess.Popen):
        try:
            assert popen.stdout is not None
            for line in popen.stdout:
                self._log(f"[{name}] {line}")
        except Exception as e:
            self._log(f"[{name}] (log reader error) {e}\n")
        finally:
            code = popen.poll()
            if code is not None:
                self._log(f"[EXIT] {name} (code={code})\n")

    def _start_process(self, name: str, cmd: str):
        full = bash_command(cmd)
        self._log(f"\n[START] {name}\n  $ {cmd}\n")
        popen = subprocess.Popen(
            full, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            preexec_fn=os.setsid,
            cwd=WORKSPACE_DIR
        )
        self.proc_handles[name] = ProcessHandle(name, popen)
        threading.Thread(target=self._reader_thread, args=(name, popen), daemon=True).start()

    def _stop_popen(self, tag: str, popen: subprocess.Popen):
        try:
            os.killpg(os.getpgid(popen.pid), signal.SIGINT)
            self._log(f"[SIGINT] {tag}\n")
        except Exception as e:
            self._log(f"[WARN] Could not SIGINT {tag}: {e}\n")

    #  modules 
    def start_selected(self):
        if self.vars["Synced node (cams + sonar)"].get():
            if self.vars["Multivideo (3 cams)"].get() or self.vars["Ping360 sonar"].get():
                messagebox.showwarning(
                    "Conflict",
                    "Synced node already includes cameras + sonar.\n"
                    "Uncheck Multivideo and Ping360 when using Synced node."
                )
                return

        selected = [name for name, v in self.vars.items() if v.get()]
        if not selected:
            messagebox.showinfo("Nothing selected", "Select at least one module.")
            return

        for name in selected:
            if name in self.proc_handles and self.proc_handles[name].popen.poll() is None:
                self._log(f"[SKIP] {name} already running.\n")
                continue
            try:
                self._start_process(name, COMMANDS[name])
            except Exception as e:
                messagebox.showerror("Start failed", str(e))
                return

    def stop_all(self):
        self.stop_recording()
        if not self.proc_handles:
            self._log("[INFO] No running module processes.\n")
            return
        self._log("\n[STOP] Stopping all module processes...\n")
        for name, h in list(self.proc_handles.items()):
            if h.popen.poll() is None:
                self._stop_popen(name, h.popen)
            self.proc_handles.pop(name, None)

    def show_status(self):
        lines = []
        for name, handle in self.proc_handles.items():
            status = "RUNNING" if handle.popen.poll() is None else f"EXIT({handle.popen.returncode})"
            lines.append(f"- {name}: {status}")
        if self.record_handle and self.record_handle.popen.poll() is None:
            lines.append("- Recording: RUNNING")
        if not lines:
            lines = ["- (no processes running)"]
        messagebox.showinfo("Status", "\n".join(lines))

    # topic detection 
    def refresh_detected_topics(self):
        """Refresh self.detected_topics from ros2 topic list -t."""
        try:
            out = run_cmd_capture("ros2 topic list -t", timeout=6)
        except Exception as e:
            self._log(f"[ERROR] Cannot list topics: {e}\n")
            return

        topics = parse_topic_list_with_types(out)
        self.detected_topics = topics
        self._log(f"[INFO] Live topics detected: {len(topics)}\n")

    def detect_cameras(self):
        """
        Fill topics_box with all live topics that look like camera/video streams.
        """
        self.refresh_detected_topics()
        if not self.detected_topics:
            messagebox.showwarning("No topics", "No topics detected. Start your camera nodes first.")
            return

        candidates = []
        for topic, typ in self.detected_topics.items():
            if typ in IMAGE_TYPES:
                name_low = topic.lower()
                # Heuristics: match common patterns
                if ("image" in name_low) or ("camera" in name_low) or ("cam" in name_low):
                    candidates.append(topic)

        candidates = unique_preserve_order(sorted(candidates))
        self._set_topics_box(candidates)
        self._log(f"[DETECT] Cameras: {len(candidates)} topics\n")

    def detect_sonar(self):
        """
        Add sonar-related topics if they exist.
        """
        self.refresh_detected_topics()
        existing = set(self.detected_topics.keys())

        found = []
        # Prefer scan_image (convertible)
        for t in ["/scan_image", "/scan", "/scan_echo"]:
            if t in existing:
                found.append(t)

        # Also include anything with sonar/ping360 keywords
        for topic, typ in self.detected_topics.items():
            name_low = topic.lower()
            if ("ping" in name_low) or ("sonar" in name_low) or ("scan" in name_low):
                found.append(topic)

        found = unique_preserve_order(found)
        self._set_topics_box(found)
        self._log(f"[DETECT] Sonar: {len(found)} topics\n")

    def detect_all(self):
        """
        Cameras + sonar + ping1d (if exists)
        """
        self.refresh_detected_topics()
        if not self.detected_topics:
            messagebox.showwarning("No topics", "No topics detected. Start your nodes first.")
            return

        existing = set(self.detected_topics.keys())

        cam = []
        for topic, typ in self.detected_topics.items():
            if typ in IMAGE_TYPES:
                name_low = topic.lower()
                if ("image" in name_low) or ("camera" in name_low) or ("cam" in name_low):
                    cam.append(topic)

        sonar = []
        for t in ["/scan_image", "/scan", "/scan_echo"]:
            if t in existing:
                sonar.append(t)
        for topic in existing:
            low = topic.lower()
            if ("ping360" in low) or ("sonar" in low) or ("scan" in low):
                sonar.append(topic)

        all_topics = unique_preserve_order(cam + sonar)
        if PING1D_TOPIC in existing:
            all_topics.append(PING1D_TOPIC)

        all_topics = unique_preserve_order(all_topics)
        self._set_topics_box(all_topics)
        self._log(f"[DETECT] All: {len(all_topics)} topics\n")

    #  recording 
    def browse_bag_out(self):
        folder = filedialog.askdirectory(title="Select folder to store bags")
        if folder:
            base = os.path.basename(self.bag_out_var.get().strip() or "session_01")
            self.bag_out_var.set(os.path.join(folder, base))

    def _get_topics_from_box(self):
        raw = self.topics_box.get("1.0", "end").strip()
        return [t.strip() for t in raw.splitlines() if t.strip()]

    def _set_topics_box(self, topics):
        self.topics_box.delete("1.0", "end")
        self.topics_box.insert("1.0", "\n".join(topics))

    def start_recording(self):
        """
        Record only topics that exist NOW.
        """
        if self.record_handle and self.record_handle.popen.poll() is None:
            self._log("[INFO] Recording already running.\n")
            return

        bag_out = self.bag_out_var.get().strip()
        storage = self.storage_var.get().strip()

        # Refresh live topics first
        self.refresh_detected_topics()
        live = set(self.detected_topics.keys())

        requested = self._get_topics_from_box()
        if not requested:
            messagebox.showerror("No topics", "Click Detect All (or Detect Cameras) first, or paste topics.")
            return

        # Keep only existing topics
        selected = [t for t in requested if t in live]
        missing = [t for t in requested if t not in live]

        if not selected:
            messagebox.showerror("No valid topics", "None of the listed topics are currently available.\nStart nodes and click Detect All.")
            return

        if missing:
            self._log("[WARN] Some topics not available, skipping:\n" + "\n".join(missing) + "\n")
            messagebox.showwarning(
                "Some topics missing",
                "Some topics are not currently published and will be skipped:\n\n" +
                "\n".join(missing)
            )

        os.makedirs(os.path.dirname(bag_out) or ".", exist_ok=True)

        # quote bag path to avoid spaces issues
        cmd = f"ros2 bag record -o '{bag_out}' --storage {storage} " + " ".join(selected)

        try:
            full = bash_command(cmd)
        except Exception as e:
            messagebox.showerror("Setup error", str(e))
            return

        self._log(f"\n[RECORD START]\n  $ {cmd}\n")
        popen = subprocess.Popen(
            full,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
            cwd=WORKSPACE_DIR
        )
        self.record_handle = ProcessHandle("Recording", popen)
        threading.Thread(target=self._reader_thread, args=("Recording", popen), daemon=True).start()

    def stop_recording(self):
        if not self.record_handle:
            return
        p = self.record_handle.popen
        if p.poll() is not None:
            self.record_handle = None
            return
        self._log("[RECORD STOP] Sending SIGINT...\n")
        self._stop_popen("Recording", p)
        self.record_handle = None

    # conversion 
    def browse_conv_bag(self):
        folder = filedialog.askdirectory(title="Select bag folder (contains metadata.yaml)")
        if folder:
            self.conv_bag_var.set(folder)

    def browse_conv_out(self):
        path = filedialog.asksaveasfilename(
            title="Save MP4 as",
            defaultextension=".mp4",
            filetypes=[("MP4 Video", "*.mp4"), ("All files", "*.*")]
        )
        if path:
            self.conv_out_var.set(path)

    def load_bag_topics_to_dropdown(self):
        bag = self.conv_bag_var.get().strip()
        storage = self.conv_storage_var.get().strip()
        if not bag:
            messagebox.showerror("Missing", "Select bag folder first.")
            return

        def worker():
            try:
                topic_map = list_bag_topics_python(bag, storage_id=storage)
                # Only show image-like topics
                img_topics = []
                for t, typ in topic_map.items():
                    if typ in IMAGE_TYPES:
                        img_topics.append(t)
                    elif typ.endswith("sensor_msgs/msg/Image") or typ.endswith("sensor_msgs/msg/CompressedImage"):
                        img_topics.append(t)
                img_topics = sorted(unique_preserve_order(img_topics))
                self._log(f"[INFO] Bag topics loaded. Image topics: {len(img_topics)}\n")

                def update_ui():
                    self.topic_dropdown["values"] = img_topics
                    if img_topics:
                        self.conv_topic_var.set(img_topics[0])
                self.after(0, update_ui)
            except Exception as e:
                self._log(f"[ERROR] Load bag topics failed: {e}\n")
                messagebox.showerror("Bag error", str(e))

        threading.Thread(target=worker, daemon=True).start()

    def start_conversion(self):
        bag = self.conv_bag_var.get().strip()
        storage = self.conv_storage_var.get().strip()
        topic = self.conv_topic_var.get().strip()
        out = self.conv_out_var.get().strip()
        fps = int(self.conv_fps_var.get())

        if not bag or not topic or not out:
            messagebox.showerror("Missing", "Select bag folder, pick a topic, and set output mp4.")
            return

        self._log(f"\n[CONVERT]\n  Bag: {bag}\n  Topic: {topic}\n  Out: {out}\n")

        def worker():
            try:
                convert_bag_to_video(bag, storage, topic, out, fps, self._log)
            except Exception as e:
                self._log(f"[ERROR] Convert failed: {e}\n")

        threading.Thread(target=worker, daemon=True).start()

    #  close 
    def on_close(self):
        running_modules = any(h.popen.poll() is None for h in self.proc_handles.values())
        running_record = self.record_handle and self.record_handle.popen.poll() is None
        if running_modules or running_record:
            if messagebox.askyesno("Exit", "Stop all running processes (including recording) and exit?"):
                self.stop_all()
                self.after(800, self.destroy)
        else:
            self.destroy()


if __name__ == "__main__":
    app = BlueROVUI()
    app.mainloop()
