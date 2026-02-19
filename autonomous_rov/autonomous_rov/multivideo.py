#!/usr/bin/env python3
import re
import glob
import time
import threading
import subprocess
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


# ============================
# GStreamer pipelines
# ============================
def gst_pipeline_udp_h264(port: int) -> str:
    return (
        f"udpsrc port={port} ! application/x-rtp, encoding-name=H264 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )

def gst_pipeline_rtsp(url: str) -> str:
    return (
        f"rtspsrc location={url} latency=120 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )

def gst_pipeline_mjpg_v4l2(device: str, width: int, height: int, fps: int) -> str:
    return (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        "jpegparse ! jpegdec ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )

def gst_pipeline_yuy2_v4l2(device: str, width: int, height: int, fps: int) -> str:
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! appsink drop=true sync=false max-buffers=1"
    )


# ============================
# Bufferless capture (latest frame)
# ============================
class BufferlessCapture:
    def __init__(self, cap: cv2.VideoCapture):
        self.cap = cap
        self.lock = threading.Lock()
        self.running = True
        self.frame = None
        self.ts = 0.0
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                now = time.time()
                with self.lock:
                    self.frame = frame
                    self.ts = now
            else:
                time.sleep(0.02)

    def read_latest(self):
        with self.lock:
            return self.frame, self.ts

    def release(self):
        self.running = False
        try:
            self.cap.release()
        except Exception:
            pass


def open_gst_capture(pipeline: str) -> BufferlessCapture:
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Failed to open GStreamer pipeline")
    return BufferlessCapture(cap)


def open_usb_capture(device: str, width: int, height: int, fps: int) -> BufferlessCapture:
    """
    Robust USB open:
      1) OpenCV direct with MJPG
      2) OpenCV direct with YUYV
      3) GStreamer MJPG
      4) GStreamer YUY2
    """
    # OpenCV direct
    cap = cv2.VideoCapture(device)
    if cap.isOpened():
        # try MJPG
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        # test a few frames
        good = 0
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                good += 1
            time.sleep(0.02)
        if good >= 2:
            return BufferlessCapture(cap)
        cap.release()

    cap = cv2.VideoCapture(device)
    if cap.isOpened():
        # try YUYV
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        good = 0
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                good += 1
            time.sleep(0.02)
        if good >= 2:
            return BufferlessCapture(cap)
        cap.release()

    # GStreamer fallback
    for pipe in [gst_pipeline_mjpg_v4l2(device, width, height, fps),
                 gst_pipeline_yuy2_v4l2(device, width, height, fps)]:
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            good = 0
            for _ in range(10):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    good += 1
                time.sleep(0.03)
            if good >= 2:
                return BufferlessCapture(cap)
            cap.release()

    raise RuntimeError(f"Failed to open USB camera {device}")


# ============================
# PC USB camera enumeration + filtering
# ============================
_CAPTURE_CACHE: Dict[str, bool] = {}

def is_video_capture_device(dev: str) -> bool:
    if dev in _CAPTURE_CACHE:
        return _CAPTURE_CACHE[dev]
    try:
        out = subprocess.check_output(["v4l2-ctl", "-d", dev, "-D"], text=True, stderr=subprocess.STDOUT)
        ok = ("Video Capture" in out) or ("Video Capture Multiplanar" in out)
        _CAPTURE_CACHE[dev] = ok
        return ok
    except Exception:
        _CAPTURE_CACHE[dev] = True
        return True


def list_v4l2_groups() -> Dict[str, List[str]]:
    """
    Group cameras by v4l2-ctl --list-devices output.
    Each group can have multiple /dev/videoX nodes; user must pick ONE that works.
    """
    groups: Dict[str, List[str]] = {}
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.STDOUT)
        current = None
        for line in out.splitlines():
            if not line.strip():
                continue
            if not line.startswith("\t") and ":" in line:
                current = line.strip().rstrip(":")
                groups[current] = []
            elif line.strip().startswith("/dev/video") and current is not None:
                groups[current].append(line.strip())
    except Exception:
        devs = sorted(glob.glob("/dev/video*"))
        groups["(all cameras)"] = devs

    # keep only capture nodes
    for k in list(groups.keys()):
        devs = [d for d in groups[k] if d.startswith("/dev/video")]
        devs = [d for d in devs if is_video_capture_device(d)]
        if devs:
            groups[k] = devs
        else:
            groups.pop(k, None)

    return groups


# ============================
# Stream discovery on BlueROV side
# ============================
def probe_gst_pipeline(pipeline: str, timeout_sec: float = 1.2) -> bool:
    ok = {"val": False}

    def worker():
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                return
            good = 0
            for _ in range(20):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    good += 1
                    if good >= 2:
                        ok["val"] = True
                        break
                time.sleep(0.05)
            cap.release()
        except Exception:
            return

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(timeout_sec)
    return ok["val"]


def discover_udp_ports(port_list: List[int], timeout_sec: float = 1.0) -> List[int]:
    found = []
    for p in port_list:
        pipe = gst_pipeline_udp_h264(p)
        if probe_gst_pipeline(pipe, timeout_sec=timeout_sec):
            found.append(p)
    return found


def discover_rtsp_streams(host: str, port: int, timeout_sec: float = 1.4) -> List[str]:
    paths = [
        "stream", "live", "camera", "cam", "video", "main", "test",
        "video_stream",
    ]
    for i in range(0, 31):
        paths.append(f"video_stream_dev_video{i}")

    urls = []
    seen = set()
    for path in paths:
        url = f"rtsp://{host}:{port}/{path}"
        if url in seen:
            continue
        seen.add(url)
        if probe_gst_pipeline(gst_pipeline_rtsp(url), timeout_sec=timeout_sec):
            urls.append(url)
    return urls


# ============================
# ROS publisher node
# ============================
@dataclass
class SelectedInput:
    name: str
    kind: str   # "usb" | "udp" | "rtsp"
    src: str    # /dev/videoX OR gst pipeline OR rtsp url


class MultiVideoPublisher(Node):
    def __init__(self, inputs: List[SelectedInput],
                 publish_fps: float, jpeg_quality: int,
                 show_preview: bool, publish_raw: bool, publish_comp: bool,
                 usb_width: int, usb_height: int, usb_fps: int):
        super().__init__("multivideo_node")
        self.bridge = CvBridge()

        self.publish_fps = float(publish_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.show_preview = bool(show_preview)
        self.publish_raw = bool(publish_raw)
        self.publish_comp = bool(publish_comp)

        self.usb_width = int(usb_width)
        self.usb_height = int(usb_height)
        self.usb_fps = int(usb_fps)

        self.captures: Dict[str, BufferlessCapture] = {}
        self.pub_raw: Dict[str, any] = {}
        self.pub_comp: Dict[str, any] = {}
        self.latest_preview: Dict[str, any] = {}

        self.get_logger().info(f"Starting inputs: {[i.name for i in inputs]}")

        for inp in inputs:
            try:
                if inp.kind == "usb":
                    cap = open_usb_capture(inp.src, self.usb_width, self.usb_height, self.usb_fps)
                elif inp.kind == "udp":
                    cap = open_gst_capture(inp.src)
                elif inp.kind == "rtsp":
                    cap = open_gst_capture(gst_pipeline_rtsp(inp.src))
                else:
                    raise RuntimeError(f"Unknown kind: {inp.kind}")
            except Exception as e:
                self.get_logger().warn(f"[SKIP] {inp.name} open failed: {e}")
                continue

            raw_topic = f"{inp.name}/image_raw"
            comp_topic = f"{inp.name}/image_raw/compressed"

            if self.publish_raw:
                self.pub_raw[inp.name] = self.create_publisher(Image, raw_topic, 10)
            if self.publish_comp:
                self.pub_comp[inp.name] = self.create_publisher(CompressedImage, comp_topic, 10)

            self.captures[inp.name] = cap
            self.get_logger().info(f"[OK] {inp.name} -> {raw_topic} + {comp_topic}")

        period = 1.0 / max(1e-6, self.publish_fps)
        self.timer = self.create_timer(period, self._tick)

        if self.show_preview:
            threading.Thread(target=self._preview_loop, daemon=True).start()

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        for name, cap in self.captures.items():
            frame, _ = cap.read_latest()
            if frame is None:
                continue

            if self.show_preview:
                self.latest_preview[name] = frame

            if self.publish_raw and name in self.pub_raw:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = stamp
                msg.header.frame_id = name
                self.pub_raw[name].publish(msg)

            if self.publish_comp and name in self.pub_comp:
                enc = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
                ok, jpg = cv2.imencode(".jpg", frame, enc)
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = stamp
                    cmsg.header.frame_id = name
                    cmsg.format = "jpeg"
                    cmsg.data = jpg.tobytes()
                    self.pub_comp[name].publish(cmsg)

    def _preview_loop(self):
        self.get_logger().info("Preview enabled. Press 'q' to close preview windows.")
        while rclpy.ok():
            shown = False
            for name, frame in list(self.latest_preview.items()):
                try:
                    cv2.imshow(name, frame)
                    shown = True
                except Exception:
                    pass
            if shown and (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
            time.sleep(0.01)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def shutdown(self):
        for cap in self.captures.values():
            try:
                cap.release()
            except Exception:
                pass
        self.captures.clear()


# ============================
# UI (USB + UDP + RTSP)
# ============================
class MultiInputUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MultiVideo - Select Inputs (PC USB + BlueROV UDP + BlueROV RTSP)")
        self.geometry("1040x820")

        # USB settings
        self.var_usb_w = tk.IntVar(value=640)
        self.var_usb_h = tk.IntVar(value=480)
        self.var_usb_fps = tk.IntVar(value=30)

        # publish settings
        self.var_pubfps = tk.DoubleVar(value=10.0)
        self.var_jpegq = tk.IntVar(value=60)
        self.var_preview = tk.BooleanVar(value=True)
        self.var_raw = tk.BooleanVar(value=True)
        self.var_comp = tk.BooleanVar(value=True)

        # UDP scan
        self.var_udp_scan = tk.BooleanVar(value=True)
        self.var_udp_ports = tk.StringVar(value="5600,5602,5604,5606")

        # RTSP scan
        self.var_rtsp_scan = tk.BooleanVar(value=True)
        self.var_rtsp_host = tk.StringVar(value="192.168.2.2")
        self.var_rtsp_port = tk.IntVar(value=8554)

        # detected
        self.usb_groups: Dict[str, List[str]] = {}
        self.usb_enable: Dict[str, tk.BooleanVar] = {}
        self.usb_choice: Dict[str, tk.StringVar] = {}

        self.udp_found: List[int] = []
        self.udp_vars: Dict[int, tk.BooleanVar] = {}

        self.rtsp_found: List[str] = []
        self.rtsp_vars: Dict[str, tk.BooleanVar] = {}

        self.user_started = False
        self.selected: Optional[List[SelectedInput]] = None

        self._build()
        self.refresh()

        self.protocol("WM_DELETE_WINDOW", self.quit_clicked)

    def _build(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        top = ttk.LabelFrame(root, text="Settings", padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="USB width").grid(row=0, column=0, sticky="w", padx=6)
        ttk.Entry(top, textvariable=self.var_usb_w, width=8).grid(row=0, column=1, sticky="w", padx=6)
        ttk.Label(top, text="USB height").grid(row=0, column=2, sticky="w", padx=6)
        ttk.Entry(top, textvariable=self.var_usb_h, width=8).grid(row=0, column=3, sticky="w", padx=6)
        ttk.Label(top, text="USB FPS").grid(row=0, column=4, sticky="w", padx=6)
        ttk.Entry(top, textvariable=self.var_usb_fps, width=8).grid(row=0, column=5, sticky="w", padx=6)

        ttk.Label(top, text="Publish FPS").grid(row=1, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Entry(top, textvariable=self.var_pubfps, width=8).grid(row=1, column=1, sticky="w", padx=6, pady=(6,0))
        ttk.Label(top, text="JPEG Q").grid(row=1, column=2, sticky="w", padx=6, pady=(6,0))
        ttk.Entry(top, textvariable=self.var_jpegq, width=8).grid(row=1, column=3, sticky="w", padx=6, pady=(6,0))

        ttk.Checkbutton(top, text="Preview windows", variable=self.var_preview).grid(row=1, column=4, sticky="w", padx=10, pady=(6,0))
        ttk.Checkbutton(top, text="Publish raw", variable=self.var_raw).grid(row=2, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Checkbutton(top, text="Publish compressed", variable=self.var_comp).grid(row=2, column=1, sticky="w", padx=6, pady=(6,0))

        net = ttk.LabelFrame(root, text="BlueROV discovery", padding=10)
        net.pack(fill="x", pady=(8,0))

        ttk.Checkbutton(net, text="Scan UDP ports", variable=self.var_udp_scan).grid(row=0, column=0, sticky="w")
        ttk.Label(net, text="Ports").grid(row=0, column=1, padx=(18,6), sticky="w")
        ttk.Entry(net, textvariable=self.var_udp_ports, width=30).grid(row=0, column=2, sticky="w")

        ttk.Checkbutton(net, text="Scan RTSP", variable=self.var_rtsp_scan).grid(row=1, column=0, sticky="w", pady=(6,0))
        ttk.Label(net, text="Host").grid(row=1, column=1, padx=(18,6), sticky="w", pady=(6,0))
        ttk.Entry(net, textvariable=self.var_rtsp_host, width=14).grid(row=1, column=2, sticky="w", pady=(6,0))
        ttk.Label(net, text="Port").grid(row=1, column=3, padx=(18,6), sticky="w", pady=(6,0))
        ttk.Entry(net, textvariable=self.var_rtsp_port, width=8).grid(row=1, column=4, sticky="w", pady=(6,0))

        self.lbl = ttk.Label(root, text="", foreground="blue")
        self.lbl.pack(fill="x", pady=(8,0))

        body = ttk.LabelFrame(root, text="Select Inputs", padding=10)
        body.pack(fill="both", expand=True, pady=(8,0))

        self.canvas = tk.Canvas(body, borderwidth=0)
        self.scroll = ttk.Scrollbar(body, orient="vertical", command=self.canvas.yview)
        self.list_frame = ttk.Frame(self.canvas)
        self.list_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.list_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scroll.pack(side="right", fill="y")

        b = ttk.Frame(root)
        b.pack(fill="x", pady=(10,0))
        ttk.Button(b, text="Refresh", command=self.refresh).pack(side="left")
        ttk.Button(b, text="Select all", command=self.select_all).pack(side="left", padx=6)
        ttk.Button(b, text="Clear all", command=self.clear_all).pack(side="left", padx=6)
        ttk.Button(b, text="Start", command=self.start_clicked).pack(side="right")
        ttk.Button(b, text="Quit", command=self.quit_clicked).pack(side="right", padx=6)

    def refresh(self):
        for w in self.list_frame.winfo_children():
            w.destroy()

        self.usb_enable.clear()
        self.usb_choice.clear()
        self.udp_vars.clear()
        self.rtsp_vars.clear()

        self.lbl.config(text="Detecting PC USB cameras + BlueROV UDP/RTSP streams...")
        self.update_idletasks()

        row = 0

        # --- BlueROV UDP ---
        ttk.Label(self.list_frame, text="BlueROV UDP streams", font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="w", pady=(6,4))
        row += 1
        if self.var_udp_scan.get():
            ports = []
            for x in self.var_udp_ports.get().split(","):
                x = x.strip()
                if x.isdigit():
                    ports.append(int(x))
            ports = sorted(list(set(ports)))
            self.udp_found = discover_udp_ports(ports, timeout_sec=0.9)
        else:
            self.udp_found = []

        if not self.udp_found:
            ttk.Label(self.list_frame, text="(none detected) — two UDP cameras need different ports").grid(row=row, column=0, sticky="w")
            row += 1
        else:
            for p in self.udp_found:
                v = tk.BooleanVar(value=False)
                self.udp_vars[p] = v
                ttk.Checkbutton(self.list_frame, text=f"Enable UDP port {p}", variable=v).grid(row=row, column=0, sticky="w")
                row += 1

        row += 1

        # --- BlueROV RTSP ---
        ttk.Label(self.list_frame, text="BlueROV RTSP streams", font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="w", pady=(6,4))
        row += 1

        if self.var_rtsp_scan.get():
            host = self.var_rtsp_host.get().strip()
            port = int(self.var_rtsp_port.get())
            self.rtsp_found = discover_rtsp_streams(host, port, timeout_sec=1.2)
        else:
            self.rtsp_found = []

        if not self.rtsp_found:
            ttk.Label(self.list_frame, text="(none detected)").grid(row=row, column=0, sticky="w")
            row += 1
        else:
            for url in self.rtsp_found:
                v = tk.BooleanVar(value=False)
                self.rtsp_vars[url] = v
                ttk.Checkbutton(self.list_frame, text=f"Enable {url}", variable=v).grid(row=row, column=0, sticky="w")
                row += 1

        row += 1

        # --- PC USB cameras ---
        ttk.Label(self.list_frame, text="PC USB cameras (you can plug in as many as you want)", font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="w", pady=(6,4))
        row += 1

        self.usb_groups = list_v4l2_groups()
        if not self.usb_groups:
            ttk.Label(self.list_frame, text="(no USB cameras detected on this PC)").grid(row=row, column=0, sticky="w")
            row += 1
        else:
            for group_name, nodes in self.usb_groups.items():
                ttk.Label(self.list_frame, text=group_name, font=("Arial", 10, "bold")).grid(row=row, column=0, sticky="w", pady=(10,2))
                row += 1

                ev = tk.BooleanVar(value=False)
                cv = tk.StringVar(value=nodes[0] if nodes else "")
                self.usb_enable[group_name] = ev
                self.usb_choice[group_name] = cv

                ttk.Checkbutton(self.list_frame, text="Enable this camera", variable=ev).grid(row=row, column=0, sticky="w")
                row += 1

                cmb = ttk.Combobox(self.list_frame, textvariable=cv, values=nodes, width=30, state="readonly")
                cmb.grid(row=row, column=0, sticky="w", pady=(0,6))
                row += 1

        self.lbl.config(text="Select your BlueROV streams and PC USB cameras, then click Start.")

    def select_all(self):
        for v in self.udp_vars.values():
            v.set(True)
        for v in self.rtsp_vars.values():
            v.set(True)
        for v in self.usb_enable.values():
            v.set(True)

    def clear_all(self):
        for v in self.udp_vars.values():
            v.set(False)
        for v in self.rtsp_vars.values():
            v.set(False)
        for v in self.usb_enable.values():
            v.set(False)

    def start_clicked(self):
        if not self.var_raw.get() and not self.var_comp.get():
            messagebox.showerror("Invalid", "Enable at least one: Publish raw or Publish compressed.")
            return

        selected: List[SelectedInput] = []

        # UDP
        idx = 0
        for p, var in self.udp_vars.items():
            if var.get():
                selected.append(SelectedInput(
                    name=f"bluerov_udp{idx}",
                    kind="udp",
                    src=gst_pipeline_udp_h264(p)
                ))
                idx += 1

        # RTSP
        idx = 0
        for url, var in self.rtsp_vars.items():
            if var.get():
                selected.append(SelectedInput(
                    name=f"bluerov_rtsp{idx}",
                    kind="rtsp",
                    src=url
                ))
                idx += 1

        # USB cameras
        idx = 0
        for group_name, ev in self.usb_enable.items():
            if not ev.get():
                continue
            dev = self.usb_choice[group_name].get().strip()
            if not dev.startswith("/dev/video"):
                continue
            selected.append(SelectedInput(
                name=f"pc_usb{idx}",
                kind="usb",
                src=dev
            ))
            idx += 1

        if not selected:
            messagebox.showerror("No selection", "Select at least one BlueROV stream or PC USB camera.")
            return

        self.selected = selected
        self.user_started = True
        self.destroy()

    def quit_clicked(self):
        self.user_started = False
        self.selected = None
        self.destroy()


def main():
    ui = MultiInputUI()
    ui.mainloop()

    if not ui.user_started or not ui.selected:
        return

    rclpy.init()
    node = MultiVideoPublisher(
        inputs=ui.selected,
        publish_fps=float(ui.var_pubfps.get()),
        jpeg_quality=int(ui.var_jpegq.get()),
        show_preview=bool(ui.var_preview.get()),
        publish_raw=bool(ui.var_raw.get()),
        publish_comp=bool(ui.var_comp.get()),
        usb_width=int(ui.var_usb_w.get()),
        usb_height=int(ui.var_usb_h.get()),
        usb_fps=int(ui.var_usb_fps.get()),
    )

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
