#!/usr/bin/env python3
"""
Linorobot2 Robot Configuration Engine - Local HTTP Server with Live Command Execution & Config Merge API
Zero-Dependency, Pure Standard Library Python 3 (Multi-Threaded)

Usage:
    python3 server.py [PORT]
    (Defaults to port 8000)
"""

import os
import sys
import json
import time
import signal
import socket
import datetime
import collections
import queue
import platform
import threading
import subprocess
import glob
import shutil
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import re

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(WEB_DIR, "..", "..", ".."))

# Add REPO_ROOT to sys.path so we can import generator and parser
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from tools.robot_config_engine.generator import generate_config_header
from tools.robot_config_engine.validator import validate_robot_spec
from tools.robot_config_engine.parser import parse_header_to_spec, merge_configurations

active_process = None
active_process_lock = threading.Lock()

class SyslogManager:
    """
    Lightweight, multi-threaded UDP Syslog server & real-time telemetry broadcaster.
    Listens on UDP (default port 514 with auto-fallback to 5140 for non-root),
    appends timestamped telemetry logs to repo logs/syslog_YYYYMMDD.log,
    and broadcasts events in real-time to Web UI SSE subscribers.
    """
    def __init__(self, repo_root):
        self.repo_root = repo_root
        self.logs_dir = os.path.join(repo_root, "logs")
        os.makedirs(self.logs_dir, exist_ok=True)
        self.sock = None
        self.thread = None
        self.is_running = False
        self.port = 514
        self.bound_port = 514
        self.packets_received = 0
        self.last_sender = ""
        self.last_message = ""
        self.last_timestamp = ""
        self.recent_logs = collections.deque(maxlen=200)
        self.subscribers = set()
        self.subscribers_lock = threading.Lock()
        self.lock = threading.Lock()

    def get_current_log_filepath(self):
        date_str = datetime.datetime.now().strftime("%Y%m%d")
        return os.path.join(self.logs_dir, f"syslog_{date_str}.log")

    def start(self, requested_port=514):
        with self.lock:
            if self.is_running:
                return {
                    "status": "already_running",
                    "port": self.bound_port,
                    "logfile": os.path.relpath(self.get_current_log_filepath(), self.repo_root),
                    "packets": self.packets_received
                }

            try:
                self.port = int(requested_port)
            except Exception:
                self.port = 514

            ports_to_try = [self.port]
            if self.port != 5140 and 5140 not in ports_to_try:
                ports_to_try.append(5140)
            if 5514 not in ports_to_try:
                ports_to_try.append(5514)

            bound = False
            last_err = None
            for p in ports_to_try:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(("0.0.0.0", p))
                    self.sock = s
                    self.bound_port = p
                    bound = True
                    break
                except Exception as e:
                    last_err = e
                    continue

            if not bound:
                raise RuntimeError(f"Could not bind UDP syslog port (tried {ports_to_try}): {last_err}")

            self.is_running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()

            return {
                "status": "running",
                "port": self.bound_port,
                "requested_port": self.port,
                "logfile": os.path.relpath(self.get_current_log_filepath(), self.repo_root),
                "packets": self.packets_received,
                "fallback": self.bound_port != self.port
            }

    def stop(self):
        with self.lock:
            if not self.is_running:
                return {"status": "stopped", "port": self.bound_port, "packets": self.packets_received}
            self.is_running = False
            if self.sock:
                try:
                    dummy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    dummy.sendto(b"__SHUTDOWN__", ("127.0.0.1", self.bound_port))
                    dummy.close()
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

            return {"status": "stopped", "port": self.bound_port, "packets": self.packets_received}

    def _listen_loop(self):
        while self.is_running:
            try:
                if not self.sock:
                    break
                data, addr = self.sock.recvfrom(4096)
                if not self.is_running:
                    break
                if data == b"__SHUTDOWN__":
                    continue

                raw_text = data.decode("utf-8", errors="replace").strip()
                now = datetime.datetime.now()
                time_str = now.strftime("%Y-%m-%d %H:%M:%S")
                client_str = f"{addr[0]}:{addr[1]}"

                pri = None
                clean_text = raw_text
                if raw_text.startswith("<") and ">" in raw_text[:6]:
                    pri_end = raw_text.index(">")
                    try:
                        pri = int(raw_text[1:pri_end])
                        clean_text = raw_text[pri_end+1:].strip()
                    except ValueError:
                        pass

                log_entry = {
                    "time": time_str,
                    "sender": client_str,
                    "ip": addr[0],
                    "port": addr[1],
                    "pri": pri,
                    "message": clean_text,
                    "raw": raw_text
                }

                formatted_line = f"[{time_str}] [{client_str}] {clean_text}\n"

                # Append to log file
                log_file = self.get_current_log_filepath()
                try:
                    with open(log_file, "a", encoding="utf-8") as f:
                        f.write(formatted_line)
                except Exception:
                    pass

                self.packets_received += 1
                self.last_sender = client_str
                self.last_message = clean_text
                self.last_timestamp = time_str
                self.recent_logs.append(log_entry)

                self._broadcast(log_entry)
            except Exception:
                if not self.is_running:
                    break
                time.sleep(0.05)

    def subscribe(self, q):
        with self.subscribers_lock:
            self.subscribers.add(q)

    def unsubscribe(self, q):
        with self.subscribers_lock:
            self.subscribers.discard(q)

    def _broadcast(self, log_entry):
        with self.subscribers_lock:
            for q in list(self.subscribers):
                try:
                    q.put_nowait(log_entry)
                except Exception:
                    pass

    def get_status(self):
        cur_file = self.get_current_log_filepath()
        file_size = os.path.getsize(cur_file) if os.path.exists(cur_file) else 0
        return {
            "running": self.is_running,
            "port": self.bound_port,
            "requested_port": self.port,
            "logfile": os.path.relpath(cur_file, self.repo_root),
            "filesize": file_size,
            "packets": self.packets_received,
            "last_sender": self.last_sender,
            "last_message": self.last_message,
            "last_timestamp": self.last_timestamp,
            "recent_count": len(self.recent_logs)
        }

syslog_manager = SyslogManager(REPO_ROOT)


def get_port_usb_details(port_path):
    port_name = os.path.basename(port_path)
    info = {
        "port": port_path,
        "vid": "",
        "pid": "",
        "product": "",
        "manufacturer": "",
        "chip": "Generic USB Serial",
        "accessible": os.access(port_path, os.R_OK | os.W_OK),
        "read": os.access(port_path, os.R_OK),
        "write": os.access(port_path, os.W_OK)
    }

    # Inspect Linux sysfs for USB Vendor & Product ID
    sys_device_path = f"/sys/class/tty/{port_name}/device"
    if os.path.exists(sys_device_path):
        curr = os.path.realpath(sys_device_path)
        for _ in range(6):
            vid_path = os.path.join(curr, "idVendor")
            pid_path = os.path.join(curr, "idProduct")
            if os.path.exists(vid_path) and os.path.exists(pid_path):
                try:
                    with open(vid_path, "r") as f:
                        info["vid"] = f.read().strip().lower()
                    with open(pid_path, "r") as f:
                        info["pid"] = f.read().strip().lower()
                    prod_path = os.path.join(curr, "product")
                    mfg_path = os.path.join(curr, "manufacturer")
                    if os.path.exists(prod_path):
                        with open(prod_path, "r") as f:
                            info["product"] = f.read().strip()
                    if os.path.exists(mfg_path):
                        with open(mfg_path, "r") as f:
                            info["manufacturer"] = f.read().strip()
                except Exception:
                    pass
                break
            curr = os.path.dirname(curr)

    # Hardware MCU / Serial Bridge Identification Matrix
    vid = info["vid"]
    pid = info["pid"]
    if vid == "2e8a":
        if pid in ["0003", "000a"]:
            info["chip"] = "Raspberry Pi Pico (RP2040)"
        elif pid in ["000f", "0005"]:
            info["chip"] = "Raspberry Pi Pico 2 (RP2350)"
        else:
            info["chip"] = "Raspberry Pi (RP2040/RP2350)"
    elif vid == "303a":
        if pid in ["1001", "1002"]:
            info["chip"] = "ESP32-S3 (Native USB CDC)"
        elif pid == "0002":
            info["chip"] = "ESP32-S2 (Native USB CDC)"
        elif pid == "1000":
            info["chip"] = "ESP32-C3 (Native USB CDC)"
        else:
            info["chip"] = "Espressif USB Serial"
    elif vid == "10c4" and pid == "ea60":
        info["chip"] = "CP2102N USB Bridge (ESP32/GenDrv)"
    elif vid == "1a86" and pid in ["7523", "5523", "7522"]:
        info["chip"] = "CH340/CH341 USB Bridge (Arduino/ESP32)"
    elif vid == "0403" and pid in ["6001", "6010", "6015"]:
        info["chip"] = "FTDI USB Serial Bridge"
    elif vid == "16c0" and pid in ["0483", "0487", "0488"]:
        info["chip"] = "Teensy USB Serial"

    return info


def get_robot_host_info():
    info = {
        "system": platform.system(),
        "node": platform.node(),
        "release": platform.release(),
        "machine": platform.machine(),
        "distro_id": "",
        "distro_name": platform.system(),
        "has_apt": shutil.which("apt-get") is not None,
        "has_dnf": shutil.which("dnf") is not None,
        "has_distrobox": shutil.which("distrobox") is not None,
        "is_container": os.path.exists("/.dockerenv") or os.path.exists("/run/.containerenv"),
        "has_pio": shutil.which("pio") is not None or os.path.exists(os.path.expanduser("~/.platformio/penv/bin/pio")),
        "installed_ros_distros": [],
        "serial_ports": []
    }
    # Check OS Release
    if os.path.exists("/etc/os-release"):
        try:
            with open("/etc/os-release", "r") as f:
                for line in f:
                    if line.startswith("ID="):
                        info["distro_id"] = line.split("=", 1)[1].strip().strip('"').lower()
                    elif line.startswith("PRETTY_NAME="):
                        info["distro_name"] = line.split("=", 1)[1].strip().strip('"')
        except Exception:
            pass

    # Check installed ROS 2 distros
    if os.path.isdir("/opt/ros"):
        try:
            info["installed_ros_distros"] = sorted(os.listdir("/opt/ros"))
        except Exception:
            pass

    # Scan attached MCU serial ports, inspect USB VID:PID, and verify access rights
    patterns = ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyTHS*", "/dev/serial0"]
    found_ports = []
    port_details = []
    seen_syspaths = set()
    for pat in patterns:
        for p in glob.glob(pat):
            found_ports.append(p)
            det = get_port_usb_details(p)
            port_details.append(det)
            if det.get("syspath"):
                try:
                    seen_syspaths.add(os.path.realpath(det["syspath"]))
                except Exception:
                    pass

    # Scan for connected USB ROM bootloaders / Mass Storage devices lacking tty drivers (e.g. RP2 Boot / BOOTSEL)
    for dev_dir in sorted(glob.glob("/sys/bus/usb/devices/*")):
        idv_f = os.path.join(dev_dir, "idVendor")
        idp_f = os.path.join(dev_dir, "idProduct")
        if os.path.exists(idv_f) and os.path.exists(idp_f):
            try:
                with open(idv_f) as f: vid = f.read().strip().lower()
                with open(idp_f) as f: pid = f.read().strip().lower()
                real_sys = os.path.realpath(dev_dir)
                # If already mapped to a tty port, skip
                if any(real_sys.startswith(s) or s.startswith(real_sys) for s in seen_syspaths):
                    continue

                prod = ""
                mfg = ""
                if os.path.exists(os.path.join(dev_dir, "product")):
                    with open(os.path.join(dev_dir, "product")) as f: prod = f.read().strip()
                if os.path.exists(os.path.join(dev_dir, "manufacturer")):
                    with open(os.path.join(dev_dir, "manufacturer")) as f: mfg = f.read().strip()

                is_bootsel = False
                chip_name = ""
                mcu_hint = ""
                if vid == "2e8a":
                    if pid in ["0003", "000a"]:
                        is_bootsel = True
                        chip_name = "Raspberry Pi Pico (RP2040) [BOOTSEL Mode]"
                        mcu_hint = "PICO"
                    elif pid in ["000f", "0005"]:
                        is_bootsel = True
                        chip_name = "Raspberry Pi Pico 2 (RP2350) [BOOTSEL Mode]"
                        mcu_hint = "PICO2"
                    else:
                        chip_name = f"Raspberry Pi RP2 Device [{vid}:{pid}]"
                        mcu_hint = "PICO"
                        is_bootsel = True
                elif vid == "303a" and pid in ["0002", "1001", "1002"]:
                    chip_name = "ESP32-S2/S3 (ROM Bootloader Mode)"
                    mcu_hint = "ESP32S3"
                    is_bootsel = True

                if is_bootsel:
                    boot_entry = {
                        "port": "BOOTSEL",
                        "accessible": True,
                        "vid": vid,
                        "pid": pid,
                        "product": prod or "RP2 Boot",
                        "manufacturer": mfg or "Raspberry Pi",
                        "chip": chip_name or f"USB Bootloader [{vid}:{pid}]",
                        "is_bootsel": True,
                        "mcu_hint": mcu_hint,
                        "post_flash_port": "/dev/ttyACM0"
                    }
                    found_ports.append("BOOTSEL")
                    port_details.append(boot_entry)
            except Exception:
                pass

    info["serial_ports"] = sorted(list(set(found_ports)))
    info["port_details"] = port_details

    # Check user dialout/plugdev group permissions
    has_dialout = False
    try:
        import grp
        user = os.environ.get("USER", "")
        if user:
            dialout_group = grp.getgrnam("dialout")
            has_dialout = user in dialout_group.gr_mem
    except Exception:
        pass
    info["has_dialout"] = has_dialout

    return info


class LinorobotEngineHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_HEAD(self):
        parsed = urlparse(self.path)
        if parsed.path in ["/api/status", "/api/configs"]:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            return
        return super().do_HEAD()

    def do_GET(self):
        parsed = urlparse(self.path)
        
        if parsed.path == "/api/syslog/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"status": "ok", **syslog_manager.get_status()}).encode("utf-8"))
            except Exception:
                pass
            return

        if parsed.path == "/api/syslog/logs":
            # List available log files and optionally read tail lines
            qs = parse_qs(parsed.query)
            tail_count = int(qs.get("tail", [50])[0])
            files = []
            if os.path.exists(syslog_manager.logs_dir):
                for f in sorted(os.listdir(syslog_manager.logs_dir), reverse=True):
                    if f.endswith(".log"):
                        fp = os.path.join(syslog_manager.logs_dir, f)
                        files.append({
                            "name": f,
                            "path": os.path.relpath(fp, REPO_ROOT),
                            "size": os.path.getsize(fp),
                            "mtime": os.path.getmtime(fp)
                        })

            current_log = syslog_manager.get_current_log_filepath()
            lines = []
            if os.path.exists(current_log):
                try:
                    with open(current_log, "r", encoding="utf-8", errors="replace") as f:
                        all_lines = f.readlines()
                        lines = [l.strip() for l in all_lines[-tail_count:]]
                except Exception:
                    pass

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({
                    "status": "ok",
                    "files": files,
                    "current_file": os.path.relpath(current_log, REPO_ROOT),
                    "lines": lines
                }).encode("utf-8"))
            except Exception:
                pass
            return

        if parsed.path == "/api/syslog/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            q = queue.Queue(maxsize=100)
            syslog_manager.subscribe(q)

            # Send initial status
            try:
                status_init = f"event: status\ndata: {json.dumps(syslog_manager.get_status())}\n\n"
                self.wfile.write(status_init.encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                syslog_manager.unsubscribe(q)
                return

            try:
                while True:
                    try:
                        entry = q.get(timeout=2.0)
                        msg = f"event: syslog\ndata: {json.dumps(entry)}\n\n"
                        self.wfile.write(msg.encode("utf-8"))
                        self.wfile.flush()
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                syslog_manager.unsubscribe(q)
            return

        if parsed.path == "/api/ros2/topics":
            # Discover active ROS 2 topics, message types, and default known rates
            topics = []
            try:
                setup_bash = ""
                for distro in ["jazzy", "lyrical", "rolling", "humble"]:
                    p = f"/opt/ros/{distro}/setup.bash"
                    if os.path.exists(p):
                        setup_bash = p
                        break
                
                cmd = f"source {setup_bash} 2>/dev/null && ros2 topic list -t"
                res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=3.5)
                if res.returncode == 0:
                    for line in res.stdout.strip().split("\n"):
                        line = line.strip()
                        if line and " [" in line and line.endswith("]"):
                            t_name, t_type = line.split(" [", 1)
                            t_name = t_name.strip()
                            t_type = t_type[:-1].strip()
                            
                            # Estimate nominal rate
                            hz_str = "--"
                            status_str = "active"
                            if "odom" in t_name:
                                hz_str = "50.0 Hz"
                            elif "imu" in t_name:
                                hz_str = "50.0 Hz"
                            elif "battery" in t_name:
                                hz_str = "1.0 Hz"
                            elif "cmd_vel" in t_name:
                                hz_str = "Subscribed"
                                status_str = "sub"
                            elif "sonar" in t_name:
                                hz_str = "20.0 Hz"
                            elif "parameter_events" in t_name or "rosout" in t_name:
                                hz_str = "Event"

                            topics.append({
                                "topic": t_name,
                                "type": t_type,
                                "hz": hz_str,
                                "status": status_str
                            })
            except Exception as e:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"status": "ok", "topics": topics}).encode("utf-8"))
            except Exception:
                pass
            return

        if parsed.path == "/api/ros2/hz_single":
            # Quick single-shot frequency measurement
            qs = parse_qs(parsed.query)
            topic = qs.get("topic", ["/odom/unfiltered"])[0].strip()
            rate_val = "0.0"
            try:
                setup_bash = ""
                for distro in ["jazzy", "lyrical", "rolling", "humble"]:
                    p = f"/opt/ros/{distro}/setup.bash"
                    if os.path.exists(p):
                        setup_bash = p
                        break
                cmd = f"source {setup_bash} 2>/dev/null && timeout 2.5 stdbuf -oL -eL ros2 topic hz {topic}"
                res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
                for line in res.stdout.split("\n"):
                    if "average rate:" in line:
                        m = re.search(r"average rate:\s*([0-9.]+)", line)
                        if m:
                            rate_val = f"{float(m.group(1)):.1f} Hz"
            except Exception:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"status": "ok", "topic": topic, "hz": rate_val}).encode("utf-8"))
            except Exception:
                pass
            return

        if parsed.path == "/api/ros2/stream":
            qs = parse_qs(parsed.query)
            topic = qs.get("topic", ["/odom/unfiltered"])[0].strip()
            mode = qs.get("mode", ["echo"])[0].strip()

            setup_bash = ""
            for distro in ["jazzy", "lyrical", "rolling", "humble"]:
                p = f"/opt/ros/{distro}/setup.bash"
                if os.path.exists(p):
                    setup_bash = p
                    break

            if mode == "hz":
                cmd = f"source {setup_bash} 2>/dev/null && stdbuf -oL -eL ros2 topic hz {topic}"
            else:
                cmd = f"source {setup_bash} 2>/dev/null && stdbuf -oL -eL ros2 topic echo {topic}"

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            init_msg = json.dumps({"type": "start", "topic": topic, "mode": mode, "text": f"--- Streaming ROS 2 topic {topic} ({mode}) ---"})
            try:
                self.wfile.write(f"data: {init_msg}\n\n".encode("utf-8"))
                self.wfile.flush()
            except Exception:
                return

            proc = None
            try:
                proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    preexec_fn=os.setsid
                )

                for line in iter(proc.stdout.readline, ""):
                    if not line:
                        break
                    line_data = json.dumps({"type": "line", "text": line.rstrip()})
                    self.wfile.write(f"data: {line_data}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except Exception:
                pass
            finally:
                if proc:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    except Exception:
                        pass
                    try:
                        proc.kill()
                    except Exception:
                        pass
            return

        if parsed.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            host_info = get_robot_host_info()
            status_data = {
                "status": "ok",
                "exec_supported": True,
                "os": host_info["system"],
                "node": host_info["node"],
                "release": host_info["release"],
                "machine": host_info["machine"],
                "distro_id": host_info["distro_id"],
                "distro_name": host_info["distro_name"],
                "has_apt": host_info["has_apt"],
                "has_dnf": host_info["has_dnf"],
                "has_distrobox": host_info["has_distrobox"],
                "is_container": host_info["is_container"],
                "has_pio": host_info["has_pio"],
                "installed_ros_distros": host_info["installed_ros_distros"],
                "serial_ports": host_info["serial_ports"],
                "port_details": host_info.get("port_details", []),
                "has_dialout": host_info.get("has_dialout", False),
                "repo_root": REPO_ROOT
            }
            try:
                self.wfile.write(json.dumps(status_data).encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if parsed.path == "/api/configs":
            # List available custom configurations in config/custom/
            custom_dir = os.path.join(REPO_ROOT, "config", "custom")
            configs = []
            if os.path.isdir(custom_dir):
                for f in sorted(os.listdir(custom_dir)):
                    if f.endswith("_config.h"):
                        full_path = os.path.join(custom_dir, f)
                        try:
                            with open(full_path, "r", encoding="utf-8") as hf:
                                content = hf.read()
                                spec = parse_header_to_spec(content)
                                configs.append({
                                    "filename": f,
                                    "path": os.path.relpath(full_path, REPO_ROOT),
                                    "robot_name": spec.get("robot_name", f[:-9]),
                                    "kinematics": spec.get("kinematics", "UNKNOWN"),
                                    "mcu": spec.get("mcu", "UNKNOWN"),
                                    "imu": spec.get("sensors", {}).get("imu_type", "UNKNOWN"),
                                    "mag": spec.get("sensors", {}).get("mag_type", "UNKNOWN"),
                                    "wifi": spec.get("telemetry", {}).get("use_wifi", False),
                                    "content": content
                                })
                        except Exception:
                            pass

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"configs": configs}).encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        try:
            return super().do_GET()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, HEAD")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        global active_process
        parsed = urlparse(self.path)

        if parsed.path == "/api/syslog/start":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            req_port = data.get("port", 514)
            try:
                res = syslog_manager.start(req_port)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps(res).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
            return

        if parsed.path == "/api/syslog/stop":
            res = syslog_manager.stop()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/parse":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                content = data.get("content", "")
                if data.get("is_json", False):
                    spec = json.loads(content)
                else:
                    spec = parse_header_to_spec(content)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "spec": spec}).encode("utf-8"))
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": f"Parse error: {e}"}).encode("utf-8"))
            return

        if parsed.path == "/api/merge":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                base_spec = data.get("base_spec", {})
                override_spec = data.get("override_spec", {})
                if not base_spec and "base_content" in data:
                    base_spec = parse_header_to_spec(data["base_content"])

                merged_spec, changes = merge_configurations(base_spec, override_spec)
                generated_header = generate_config_header(merged_spec)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "ok",
                    "merged_spec": merged_spec,
                    "changes": changes,
                    "header": generated_header
                }).encode("utf-8"))
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": f"Merge error: {e}"}).encode("utf-8"))
            return

        if parsed.path == "/api/save":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                filename = data.get("filename", "")
                header_content = data.get("header", "")
                if not filename:
                    robot_name = data.get("spec", {}).get("robot_name", "custom")
                    filename = f"{robot_name}_config.h"

                save_path = os.path.join(REPO_ROOT, "config", "custom", filename)
                os.makedirs(os.path.dirname(save_path), exist_ok=True)
                with open(save_path, "w", encoding="utf-8") as sf:
                    sf.write(header_content)

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "saved",
                    "path": os.path.relpath(save_path, REPO_ROOT),
                    "filename": filename
                }).encode("utf-8"))
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": f"Save error: {e}"}).encode("utf-8"))
            return

        if parsed.path == "/api/kill":
            with active_process_lock:
                if active_process and active_process.poll() is None:
                    try:
                        active_process.terminate()
                        time.sleep(0.2)
                        if active_process.poll() is None:
                            active_process.kill()
                    except Exception:
                        pass
                    active_process = None

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"status": "killed"}).encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if parsed.path == "/api/exec":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": f"Invalid JSON: {e}"}).encode("utf-8"))
                return

            command = data.get("command", "").strip()
            cwd = data.get("cwd", REPO_ROOT)
            if not os.path.isabs(cwd):
                cwd = os.path.join(REPO_ROOT, cwd)

            if not command:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": "Empty command"}).encode("utf-8"))
                return

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            penv_path = os.path.expanduser("~/.platformio/penv/bin")
            local_bin = os.path.expanduser("~/.local/bin")
            env["PATH"] = f"{penv_path}:{local_bin}:{env.get('PATH', '')}"
            if "ROS_DISTRO" not in env or not env["ROS_DISTRO"]:
                env["ROS_DISTRO"] = "jazzy"

            def send_event(event_type, payload):
                msg = f"event: {event_type}\ndata: {json.dumps(payload)}\n\n"
                try:
                    self.wfile.write(msg.encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass

            # Adaptive execution: If host lacks /opt/ros but distrobox has jazzy container, route commands into distrobox
            has_distrobox = shutil.which("distrobox") is not None
            has_host_ros = os.path.exists("/opt/ros")
            if (not has_host_ros) and has_distrobox and ("ros2" in command or "source /opt/ros" in command):
                escaped_cmd = command.replace('"', '\\"')
                command = f'distrobox enter jazzy -- bash -c "source /opt/ros/jazzy/setup.bash 2>/dev/null; [ -f /home/ubuntu/box/jazzy/uros_ws/install/setup.bash ] && source /home/ubuntu/box/jazzy/uros_ws/install/setup.bash; {escaped_cmd}"'

            send_event("start", {"command": command, "cwd": cwd})

                        # Proactively stop serial-transport micro-ROS agent and free USB port if flashing (WiFi agent kept running)
            if any(k in command for k in ["-t upload", "picotool", "flash", "deploy"]):
                try:
                    subprocess.run(["pkill", "-f", "micro_ros_agent.*serial"], capture_output=True, timeout=2.0)
                    subprocess.run(["pkill", "-f", "micro_ros_agent.*multiserial"], capture_output=True, timeout=2.0)
                    subprocess.run(["pkill", "-f", "miniterm"], capture_output=True, timeout=2.0)
                except Exception:
                    pass

            try:
                with active_process_lock:
                    active_process = subprocess.Popen(
                        command,
                        cwd=cwd,
                        shell=True,
                        executable="/bin/bash",
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1,
                        env=env,
                        preexec_fn=os.setsid if hasattr(os, "setsid") else None
                    )

                if active_process and active_process.stdout:
                    for line in iter(active_process.stdout.readline, ""):
                        if not line:
                            break
                        send_event("output", {"line": line, "text": line})

                active_process.wait()
                exit_code = active_process.returncode
                send_event("done", {"code": exit_code, "exit_code": exit_code})
            except Exception as e:
                send_event("error", {"error": str(e)})
            finally:
                with active_process_lock:
                    active_process = None
            return

        self.send_response(404)
        self.end_headers()


if __name__ == "__main__":
    print(f"============================================================")
    print(f" Linorobot2 Robot Configuration Engine & Web Studio")
    print(f" Live HTTP Server listening on port {PORT}")
    print(f" Web UI: http://localhost:{PORT}")
    print(f" Repository Root: {REPO_ROOT}")
    print(f" Press Ctrl+C to terminate.")
    print(f"============================================================")
    server = ThreadingHTTPServer(("0.0.0.0", PORT), LinorobotEngineHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.server_close()
