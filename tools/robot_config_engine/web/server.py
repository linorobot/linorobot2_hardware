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

PORT = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 8000


def running_in_container():
    """True when this server itself runs inside a container (distrobox, podman, docker).

    Nested `distrobox enter` is not supported, so when we are already inside a
    container we must run commands directly in our own environment.
    """
    if os.environ.get("CONTAINER_ID") or os.environ.get("DISTROBOX_ENTER_PATH"):
        return True
    return os.path.exists("/run/.containerenv") or os.path.exists("/.dockerenv")


IN_CONTAINER = running_in_container()

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(WEB_DIR, "..", "..", ".."))

# Add REPO_ROOT to sys.path so we can import generator and parser
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from tools.robot_config_engine.generator import generate_config_header
from tools.robot_config_engine.validator import validate_robot_spec
from tools.robot_config_engine.parser import parse_header_to_spec, merge_configurations, merge_wifi_config

active_process = None
active_process_lock = threading.Lock()


def _wifi_config_content():
    """Contents of the git-ignored config/custom/wifi_config.h, or '' if absent."""
    p = os.path.join(REPO_ROOT, "config", "custom", "wifi_config.h")
    try:
        with open(p, "r", encoding="utf-8") as f:
            return f.read()
    except OSError:
        return ""


def _git(*args, cwd=REPO_ROOT):
    """Run `git <args>` in the repo; return stripped stdout, or '' on any failure."""
    try:
        out = subprocess.run(
            ["git", *args], cwd=cwd, capture_output=True, text=True, timeout=5.0
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return ""


def collect_git_info():
    """Snapshot of the checked-out repo: 7-char commit, branch, remotes, last 10 commits."""
    remotes = []
    for line in _git("remote", "-v").splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] == "(fetch)":
            remotes.append({"name": parts[0], "url": parts[1]})
    commits = []
    log = _git("log", "-10", "--pretty=format:%h\x1f%s\x1f%an\x1f%ad\x1f%ar", "--date=short")
    for line in log.splitlines():
        f = line.split("\x1f")
        if len(f) == 5:
            commits.append({
                "hash": f[0], "subject": f[1], "author": f[2],
                "date": f[3], "reldate": f[4],
            })
    cur_branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "(detached)"
    # Local branches, most-recently-committed first, current branch pinned to the top.
    branches = [
        b for b in _git(
            "for-each-ref", "--sort=-committerdate",
            "--format=%(refname:short)", "refs/heads"
        ).splitlines() if b
    ]
    if cur_branch in branches:
        branches = [cur_branch] + [b for b in branches if b != cur_branch]
    return {
        "version": (_git("rev-parse", "--short=7", "HEAD") or "unknown")[:7],
        "full": _git("rev-parse", "HEAD"),
        "branch": cur_branch,
        "branches": branches,
        "dirty": bool(_git("status", "--porcelain")),
        "remotes": remotes,
        "commits": commits,
    }


# The commit the web server was started on ("the version we start the web").
GIT_VERSION_AT_START = (_git("rev-parse", "--short=7", "HEAD") or "unknown")[:7]


# Rolling cap for the on-disk syslog capture: a chatty robot streaming for days
# would otherwise fill the disk. At the limit the file is rotated to a single
# .1 backup (so at most ~2x this per day).
SYSLOG_MAX_BYTES = 10 * 1024 * 1024


class SyslogManager:
    """
    Lightweight, multi-threaded UDP Syslog server & real-time telemetry broadcaster.
    Listens on UDP (default port 514 with auto-fallback to 5140 for non-root),
    appends timestamped telemetry logs to repo logs/syslog_YYYYMMDD.log (rotated
    at SYSLOG_MAX_BYTES with one .1 backup), and broadcasts events in real-time
    to Web UI SSE subscribers.
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
        # Persistent append handle for the current log file (avoids an open()
        # per packet and lets us track size for rotation).
        self._log_fh = None
        self._log_path = None
        self._log_bytes = 0

    def get_current_log_filepath(self):
        date_str = datetime.datetime.now().strftime("%Y%m%d")
        return os.path.join(self.logs_dir, f"syslog_{date_str}.log")

    def _close_log(self):
        if self._log_fh:
            try:
                self._log_fh.close()
            except Exception:
                pass
        self._log_fh = None
        self._log_path = None
        self._log_bytes = 0

    def _write_log_line(self, line):
        """Append one line, opening/rotating the day's log file as needed."""
        path = self.get_current_log_filepath()
        try:
            if self._log_fh is None or self._log_path != path:
                # First write, or the date rolled over past midnight.
                self._close_log()
                self._log_path = path
                self._log_fh = open(path, "a", encoding="utf-8")
                try:
                    self._log_bytes = os.path.getsize(path)
                except OSError:
                    self._log_bytes = 0

            encoded_len = len(line.encode("utf-8"))
            if self._log_bytes + encoded_len > SYSLOG_MAX_BYTES:
                # Rotate: current -> <name>.1 (replacing any previous backup).
                self._log_fh.close()
                try:
                    os.replace(path, path + ".1")
                except OSError:
                    pass
                self._log_fh = open(path, "a", encoding="utf-8")
                self._log_bytes = 0

            self._log_fh.write(line)
            self._log_fh.flush()
            self._log_bytes += encoded_len
        except Exception:
            self._close_log()

    def start(self, requested_port=514):
        with self.lock:
            if self.is_running:
                return {
                    "status": "already_running",
                    "running": True,
                    "host_ip": self._host_ip(),
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
                "running": True,
                "host_ip": self._host_ip(),
                "port": self.bound_port,
                "requested_port": self.port,
                "logfile": os.path.relpath(self.get_current_log_filepath(), self.repo_root),
                "packets": self.packets_received,
                "fallback": self.bound_port != self.port
            }

    def stop(self):
        with self.lock:
            if not self.is_running:
                return {"status": "stopped", "running": False, "port": self.bound_port, "packets": self.packets_received}
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
            self._close_log()

            return {"status": "stopped", "running": False, "port": self.bound_port, "packets": self.packets_received}

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

                # Append to log file (rotates at SYSLOG_MAX_BYTES)
                self._write_log_line(formatted_line)

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

    @staticmethod
    def _host_ip():
        """Best-effort primary LAN IP of this host (no traffic actually sent)."""
        import socket as _s
        for probe in ("10.255.255.255", "192.168.255.255", "8.8.8.8"):
            try:
                k = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
                k.connect((probe, 1)); ip = k.getsockname()[0]; k.close()
                if ip and not ip.startswith("127."):
                    return ip
            except Exception:
                pass
        try:
            return _s.gethostbyname(_s.gethostname())
        except Exception:
            return "127.0.0.1"

    def get_status(self):
        cur_file = self.get_current_log_filepath()
        file_size = os.path.getsize(cur_file) if os.path.exists(cur_file) else 0
        return {
            "running": self.is_running,
            "host_ip": self._host_ip(),
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


def _mcu_from_platformio(platform: str, board: str, env: str = "") -> str:
    """MCU family from platformio.ini `platform =` / `board =` (authoritative)."""
    b = (board or "").lower().strip()
    p = (platform or "").lower()
    e = (env or "").lower()
    if b.startswith("teensy") or "teensy" in p:
        return "TEENSY"
    if b in ("rpipico2w",) or "pico2w" in b:
        return "PICO2W"
    if b in ("rpipico2",) or "pico2" in b or "rp2350" in b:
        return "PICO2"
    if b in ("rpipicow",) or "picow" in b:
        return "PICOW"
    if b in ("rpipico",) or "pico" in b or "rp2040" in b:
        return "PICO"
    if "esp32-s3" in b or "esp32s3" in b:
        return "ESP32S3"
    if "esp32-s2" in b or "esp32s2" in b:
        return "ESP32S2"
    if "espressif32" in p or b in ("nodemcu-32s",) or "esp32" in b:
        return "GENDRV" if e.startswith("gendrv") else "ESP32"
    return ""


def _parse_platformio_boards(ini_path: str) -> list:
    """
    Parse firmware/platformio.ini and return a list of board environment dicts:
      env, config_file, config_macro, transport, wifi, board, monitor_speed, display_name
    Only environments with a USE_*_CONFIG define in build_flags are included.
    """
    # Macro → relative path from REPO_ROOT
    MACRO_TO_CONFIG = {
        "USE_GENDRV_CONFIG":  "config/custom/gendrv_config.h",
        "USE_ESP32_CONFIG":   "config/custom/esp32_config.h",
        "USE_ESP32S2_CONFIG": "config/custom/esp32s2_config.h",
        "USE_ESP32S3_CONFIG": "config/custom/esp32s3_config.h",
        "USE_PICO_CONFIG":    "config/custom/pico_config.h",
        "USE_VATTENKAR_CONFIG": "config/custom/vattenkar_config.h",
        "USE_TEENSY_CONFIG":  "config/lino_base_config.h",
    }
    # Friendly display names
    ENV_DISPLAY = {
        "teensy41": "Teensy 4.1",
        "teensy40": "Teensy 4.0",
        "teensy36": "Teensy 3.6",
        "teensy35": "Teensy 3.5",
        "teensy31": "Teensy 3.1 / 3.2",
        "vattenkar": "Vattenkar (Teensy 4.0)",
        "gendrv": "Waveshare General Driver (ESP32, Serial)",
        "gendrv_wifi": "Waveshare General Driver (ESP32, WiFi)",
        "esp32": "ESP32 DevKit (Serial)",
        "esp32_wifi": "ESP32 DevKit (WiFi)",
        "esp32s2": "ESP32-S2 (Serial)",
        "esp32s2_wifi": "ESP32-S2 (WiFi)",
        "esp32s3": "ESP32-S3 (Serial)",
        "esp32s3_wifi": "ESP32-S3 (WiFi)",
        "pico": "Raspberry Pi Pico (RP2040, Serial)",
        "picow": "Raspberry Pi Pico W (RP2040, Serial)",
        "picow_wifi": "Raspberry Pi Pico W (RP2040, WiFi)",
        "pico2": "Raspberry Pi Pico 2 (RP2350, Serial)",
        "pico2w": "Raspberry Pi Pico 2W (RP2350, Serial)",
        "pico2w_wifi": "Raspberry Pi Pico 2W (RP2350, WiFi)",
    }

    if not os.path.isfile(ini_path):
        return []

    with open(ini_path, "r", encoding="utf-8") as f:
        raw = f.read()

    # Split into sections
    section_re = re.compile(r'^\[env:([^\]]+)\]', re.MULTILINE)
    positions = [(m.start(), m.group(1)) for m in section_re.finditer(raw)]

    boards = []
    for idx, (pos, env_name) in enumerate(positions):
        end = positions[idx + 1][0] if idx + 1 < len(positions) else len(raw)
        section_text = raw[pos:end]

        # Extract build_flags block (multi-line, continuation lines indented)
        bf_match = re.search(r'build_flags\s*=\s*((?:.*\n(?:[ \t]+.*\n)*)*)', section_text)
        build_flags = bf_match.group(1) if bf_match else ""

        # Find USE_*_CONFIG macro
        macro_match = re.search(r'-D\s+(USE_\w+_CONFIG)', build_flags)
        if not macro_match:
            # Teensy boards use generic config (no USE_*_CONFIG)
            # They inherit from [env] which has no USE_*_CONFIG, but we can
            # identify them by board type
            board_val = re.search(r'^board\s*=\s*(\S+)', section_text, re.MULTILINE)
            bv = board_val.group(1) if board_val else ""
            if "teensy" in bv or "vattenkar" in env_name.lower():
                config_macro = "USE_TEENSY_CONFIG"
                config_file = MACRO_TO_CONFIG.get("USE_TEENSY_CONFIG", "config/lino_base_config.h")
            else:
                continue
        else:
            config_macro = macro_match.group(1)
            config_file = MACRO_TO_CONFIG.get(config_macro, "")
            if not config_file:
                # Generic: USE_<NAME>_CONFIG -> config/custom/<name>_config.h
                stem = config_macro[len("USE_"):-len("_CONFIG")].lower() if config_macro.startswith("USE_") and config_macro.endswith("_CONFIG") else ""
                for cand in (stem, env_name.lower()):
                    if cand and os.path.isfile(os.path.join(REPO_ROOT, "config", "custom", f"{cand}_config.h")):
                        config_file = f"config/custom/{cand}_config.h"
                        break

        # Transport detection
        wifi = (
            re.search(r'^board_microros_transport\s*=\s*wifi', section_text, re.MULTILINE) is not None
            or "-D USE_WIFI" in build_flags
        )
        transport = "WIFI_UDP" if wifi else "SERIAL"

        # Board hardware ID + platform
        board_hw = ""
        bm = re.search(r'^board\s*=\s*(\S+)', section_text, re.MULTILINE)
        if bm:
            board_hw = bm.group(1)
        platform_hw = ""
        pm = re.search(r'^platform\s*=\s*(\S+)', section_text, re.MULTILINE)
        if pm:
            platform_hw = pm.group(1)
        # env `[env]` section has no `board` (teensy default lives there) —
        # fall back to the file-level platform for teensy child envs.
        if not platform_hw and "teensy" in board_hw:
            platform_hw = "teensy"

        # Monitor speed
        monitor_speed = None
        ms_m = re.search(r'^monitor_speed\s*=\s*(\d+)', section_text, re.MULTILINE)
        if ms_m:
            monitor_speed = int(ms_m.group(1))

        # Only include if config file exists on disk
        config_exists = bool(config_file) and os.path.isfile(os.path.join(REPO_ROOT, config_file))

        boards.append({
            "env":           env_name,
            "display_name":  ENV_DISPLAY.get(env_name, env_name),
            "mcu":           _mcu_from_platformio(platform_hw, board_hw, env_name),
            "platform":      platform_hw,
            "config_macro":  config_macro,
            "config_file":   config_file,
            "config_exists": config_exists,
            "transport":     transport,
            "wifi":          wifi,
            "board":         board_hw,
            "monitor_speed": monitor_speed,
        })

    return boards



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
        # CP2102 and CP2102N share this VID:PID by default — only the USB
        # product string tells them apart. CP2102N (rated/verified up to
        # 1.5M+ baud) only gets the fast label when explicitly confirmed;
        # a plain CP2102 (max ~1 Mbps per datasheet) stays conservative.
        if "cp2102n" in info["product"].lower():
            info["chip"] = "CP2102N USB Bridge (ESP32/GenDrv)"
        else:
            info["chip"] = "CP2102 USB Bridge (ESP32)"
    elif vid == "1a86" and pid in ["7523", "5523", "7522"]:
        info["chip"] = "CH340/CH341 USB Bridge (Arduino/ESP32)"
    elif vid == "0403" and pid in ["6001", "6010", "6015"]:
        info["chip"] = "FTDI USB Serial Bridge"
    elif vid == "16c0" and pid in ["0483", "0487", "0488"]:
        info["chip"] = "Teensy USB Serial"

    return info


def check_container_status():
    has_docker = shutil.which("docker") is not None
    has_podman = shutil.which("podman") is not None
    has_builder = False
    has_flasher = False
    is_rootless_docker = False

    if has_docker:
        try:
            info_res = subprocess.run(
                ["docker", "info", "-f", "{{.SecurityOptions}}"],
                capture_output=True, text=True, timeout=3
            )
            if "rootless" in (info_res.stdout or "").lower():
                is_rootless_docker = True
            elif "docker.sock" in os.environ.get("DOCKER_HOST", "") and "run/user" in os.environ.get("DOCKER_HOST", ""):
                is_rootless_docker = True
            else:
                res_u = subprocess.run(
                    ["systemctl", "--user", "is-active", "docker"],
                    capture_output=True, text=True, timeout=2
                )
                if res_u.stdout.strip() == "active":
                    is_rootless_docker = True
        except Exception:
            pass

        try:
            res_b = subprocess.run(["docker", "images", "-q", "linorobot2-builder:latest"], capture_output=True, text=True, timeout=3)
            has_builder = bool(res_b.stdout.strip())
            res_f = subprocess.run(["docker", "images", "-q", "linorobot2-flasher:latest"], capture_output=True, text=True, timeout=3)
            has_flasher = bool(res_f.stdout.strip())
        except Exception:
            pass

    if has_podman and not has_builder:
        try:
            res_pb = subprocess.run(["podman", "images", "-q", "linorobot2-builder:latest"], capture_output=True, text=True, timeout=3)
            has_builder = bool(res_pb.stdout.strip())
            res_pf = subprocess.run(["podman", "images", "-q", "linorobot2-flasher:latest"], capture_output=True, text=True, timeout=3)
            has_flasher = bool(res_pf.stdout.strip())
        except Exception:
            pass

    return {
        "status": "ok",
        "has_docker": has_docker,
        "is_rootless_docker": is_rootless_docker,
        "has_podman": has_podman,
        "has_builder_image": has_builder,
        "has_flasher_image": has_flasher,
        "platform_system": platform.system()
    }


def get_rootless_info():
    c_status = check_container_status()
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    user = os.environ.get("USER", "user")
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    return {
        "status": "ok",
        "is_rootless": c_status["is_rootless_docker"],
        "has_docker": c_status["has_docker"],
        "has_podman": c_status["has_podman"],
        "platform_system": c_status["platform_system"],
        "user": user,
        "uid": uid,
        "xdg_runtime_dir": xdg_runtime,
        "commands": {
            "ubuntu_debian": [
                "sudo apt-get update && sudo apt-get install -y uidmap dbus-user-session slirp4netns",
                "dockerd-rootless-setuptool.sh install",
                "systemctl --user enable --now docker.service",
                f"loginctl enable-linger {user}",
                f'export DOCKER_HOST="unix://{xdg_runtime}/docker.sock"'
            ],
            "fedora": [
                "sudo dnf install -y shadow-utils-subid docker-ce-rootless-extras slirp4netns fuse-overlayfs",
                "dockerd-rootless-setuptool.sh install",
                "systemctl --user enable --now docker.service",
                f"loginctl enable-linger {user}",
                f'export DOCKER_HOST="unix://{xdg_runtime}/docker.sock"'
            ],
            "podman_alternative": [
                "sudo apt-get install -y podman  # or: sudo dnf install -y podman",
                "podman run --version"
            ]
        }
    }




def setup_rootless_docker():
    logs = []
    user = os.environ.get("USER", "ubuntu")
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    if platform.system() != "Linux":
        return {
            "status": "ok",
            "success": True,
            "message": "Rootless setup is only needed on Linux. Windows and macOS map volume permissions automatically.",
            "is_rootless": True,
            "logs": "Non-Linux platform."
        }

    status = check_container_status()
    if status["is_rootless_docker"]:
        return {
            "status": "ok",
            "success": True,
            "message": f"Rootless Docker is already active for user '{user}' (UID {uid}).",
            "is_rootless": True,
            "logs": "Rootless daemon is already active."
        }

    setuptool = shutil.which("dockerd-rootless-setuptool.sh")
    if not setuptool:
        for p in ["/usr/bin/dockerd-rootless-setuptool.sh", os.path.expanduser("~/.docker/bin/dockerd-rootless-setuptool.sh")]:
            if os.path.exists(p):
                setuptool = p
                break

    if not setuptool:
        logs.append("Attempting to install rootless Docker prerequisites...")
        if shutil.which("apt-get"):
            subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True)
            r_inst = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "uidmap", "dbus-user-session", "slirp4netns", "docker-ce-rootless-extras"], capture_output=True, text=True)
            logs.append(r_inst.stdout or r_inst.stderr)
        elif shutil.which("dnf"):
            r_inst = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "shadow-utils-subid", "docker-ce-rootless-extras", "slirp4netns", "fuse-overlayfs"], capture_output=True, text=True)
            logs.append(r_inst.stdout or r_inst.stderr)
        setuptool = shutil.which("dockerd-rootless-setuptool.sh")

    if not setuptool:
        return {
            "status": "error",
            "success": False,
            "message": "dockerd-rootless-setuptool.sh not found. Install uidmap and docker-ce-rootless-extras.",
            "is_rootless": False,
            "logs": "\n".join(logs)
        }

    logs.append("Running dockerd-rootless-setuptool.sh install -f...")
    try:
        r_install = subprocess.run([setuptool, "install", "-f"], capture_output=True, text=True, timeout=30)
        logs.append(r_install.stdout)
        if r_install.stderr:
            logs.append(r_install.stderr)
    except Exception as e:
        logs.append(f"Execution error: {e}")

    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "enable", "--now", "docker.service"], capture_output=True, text=True)
    subprocess.run(["loginctl", "enable-linger", user], capture_output=True, text=True)

    docker_sock = f"unix://{xdg_runtime}/docker.sock"
    os.environ["DOCKER_HOST"] = docker_sock

    bashrc = os.path.expanduser("~/.bashrc")
    try:
        if os.path.exists(bashrc):
            with open(bashrc, "r", encoding="utf-8") as f:
                b_cnt = f.read()
            if "DOCKER_HOST" not in b_cnt:
                with open(bashrc, "a", encoding="utf-8") as f:
                    f.write(f'\nexport DOCKER_HOST="{docker_sock}"\n')
                logs.append("Appended DOCKER_HOST to ~/.bashrc")
    except Exception as e:
        logs.append(f"Notice: could not update ~/.bashrc: {e}")

    new_status = check_container_status()
    success = new_status["is_rootless_docker"]

    return {
        "status": "ok" if success else "warning",
        "success": success,
        "is_rootless": success,
        "message": f"Rootless Docker {'configured and active' if success else 'setup completed with warnings'}.",
        "logs": "\n".join(logs)
    }


def install_container_engine(engine="docker"):
    logs = []
    engine = engine.lower().strip()

    if platform.system() != "Linux":
        return {
            "status": "ok",
            "installed": True,
            "engine": engine,
            "message": "On Windows/macOS, please install Docker Desktop or Podman Desktop.",
            "logs": "Non-Linux platform."
        }

    has_apt = shutil.which("apt-get") is not None
    has_dnf = shutil.which("dnf") is not None

    if engine in ("podman", "podman_systemd"):
        if shutil.which("podman"):
            return {
                "status": "ok",
                "installed": True,
                "engine": "podman",
                "message": "Podman is already installed.",
                "logs": "podman binary present."
            }
        logs.append("Installing Podman...")
        if has_apt:
            subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True, timeout=60)
            r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "podman"], capture_output=True, text=True, timeout=180)
            logs.append(r.stdout or r.stderr)
        elif has_dnf:
            r = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "podman"], capture_output=True, text=True, timeout=180)
            logs.append(r.stdout or r.stderr)
        else:
            return {
                "status": "error",
                "installed": False,
                "engine": "podman",
                "message": "No supported package manager found (apt-get or dnf).",
                "logs": "Unsupported package manager."
            }

        installed = shutil.which("podman") is not None
        return {
            "status": "ok" if installed else "error",
            "installed": installed,
            "engine": "podman",
            "message": "Podman installed successfully." if installed else "Podman installation failed.",
            "logs": "\n".join(logs)
        }

    elif engine == "docker":
        if not shutil.which("docker"):
            logs.append("Installing Docker packages...")
            if has_apt:
                subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True, timeout=60)
                r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "docker.io", "uidmap", "dbus-user-session", "slirp4netns"], capture_output=True, text=True, timeout=180)
                logs.append(r.stdout or r.stderr)
            elif has_dnf:
                r = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "docker-ce", "shadow-utils-subid", "docker-ce-rootless-extras", "slirp4netns", "fuse-overlayfs"], capture_output=True, text=True, timeout=180)
                logs.append(r.stdout or r.stderr)
            else:
                return {
                    "status": "error",
                    "installed": False,
                    "engine": "docker",
                    "message": "No supported package manager found (apt-get or dnf).",
                    "logs": "Unsupported package manager."
                }

        rootless_res = setup_rootless_docker()
        logs.append(rootless_res.get("logs", ""))

        status = check_container_status()
        installed = status["has_docker"]
        return {
            "status": "ok" if installed else "error",
            "installed": installed,
            "is_rootless": status["is_rootless_docker"],
            "engine": "docker",
            "message": "Docker (Rootless) installed and configured." if (installed and status["is_rootless_docker"]) else ("Docker installed." if installed else "Docker installation failed."),
            "logs": "\n".join(logs)
        }

    return {
        "status": "error",
        "installed": False,
        "engine": engine,
        "message": f"Unknown container engine '{engine}'.",
        "logs": "Invalid engine requested."
    }


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



# =============================================================================
# Workflow & Board Setup Persistence
# =============================================================================
WORKFLOW_SETTINGS_PATH = os.path.join(REPO_ROOT, "config", "custom", ".workflow_settings.json")

DEFAULT_WORKFLOW_SETTINGS = {
    "ros_distro": "jazzy",
    "build_engine": "docker",
    "agent_engine": "docker",
    "container_registry": "auto",
    "custom_registry": "",
    "remote_flash": "",
    "flash_port": "",
    "multi_ports": "",
    "robot_name": "scout_pico2",
    "branch": "scout_pico2",
    "preset": "bare",
    "board_env": "",
    "spec": {}
}

def load_workflow_settings():
    if os.path.exists(WORKFLOW_SETTINGS_PATH):
        try:
            with open(WORKFLOW_SETTINGS_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                res = DEFAULT_WORKFLOW_SETTINGS.copy()
                res.update(data)
                return res
        except Exception:
            pass
    return DEFAULT_WORKFLOW_SETTINGS.copy()

def save_workflow_settings(data):
    os.makedirs(os.path.dirname(WORKFLOW_SETTINGS_PATH), exist_ok=True)
    curr = load_workflow_settings()
    curr.update(data)
    with open(WORKFLOW_SETTINGS_PATH, "w", encoding="utf-8") as f:
        json.dump(curr, f, indent=2)
    return curr


# =============================================================================
# Remote Target & SSH Management
# =============================================================================
REMOTE_CONFIG_PATH = os.path.join(REPO_ROOT, "config", "custom", ".remote_config.json")

DEFAULT_REMOTE_CONFIG = {
    "target_mode": "local",  # "local" or "remote"
    "remote_host": "192.168.1.100",
    "remote_user": "ubuntu",
    "remote_port": 22,
    "remote_key": "",
    "remote_serial_port": "/dev/ttyUSB0",
    "remote_baud": 1500000
}

def load_remote_config():
    if os.path.exists(REMOTE_CONFIG_PATH):
        try:
            with open(REMOTE_CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                res = DEFAULT_REMOTE_CONFIG.copy()
                res.update(data)
                return res
        except Exception:
            pass
    return DEFAULT_REMOTE_CONFIG.copy()

def save_remote_config(data):
    os.makedirs(os.path.dirname(REMOTE_CONFIG_PATH), exist_ok=True)
    curr = load_remote_config()
    curr.update(data)
    with open(REMOTE_CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(curr, f, indent=2)
    return curr

def build_ssh_args(cfg):
    user = (cfg.get("remote_user") or "ubuntu").strip()
    host = (cfg.get("remote_host") or "127.0.0.1").strip()
    port = str(cfg.get("remote_port") or 22).strip()
    key = (cfg.get("remote_key") or "").strip()
    args = [
        "ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=5",
        "-o", "StrictHostKeyChecking=no",
        "-p", port
    ]
    if key and os.path.exists(os.path.expanduser(key)):
        args.extend(["-i", os.path.expanduser(key)])
    args.append(f"{user}@{host}")
    return args

def test_remote_ssh(cfg):
    args = build_ssh_args(cfg)
    check_script = (
        "echo 'SSH_ALIVE'; "
        "which esptool.py 2>/dev/null || python3 -m esptool version 2>/dev/null || echo 'NO_ESPTOOL'; "
        "which picotool 2>/dev/null || echo 'NO_PICOTOOL'; "
        "which docker 2>/dev/null || echo 'NO_DOCKER'; "
        "uname -s -r -m; "
        "cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | head -n 1"
    )
    args.append(check_script)
    try:
        res = subprocess.run(args, capture_output=True, text=True, timeout=8)
        if res.returncode != 0 and "SSH_ALIVE" not in res.stdout:
            err = res.stderr.strip() or f"SSH exited with code {res.returncode}"
            return {"ok": False, "error": err}

        stdout = res.stdout
        has_docker = "NO_DOCKER" not in stdout and "docker" in stdout
        has_esptool = "NO_ESPTOOL" not in stdout and ("esptool" in stdout or "5." in stdout or "4." in stdout)
        has_picotool = "NO_PICOTOOL" not in stdout and "picotool" in stdout

        # Docker Anywhere: auto-install Docker on Robot PC if missing
        if not has_docker:
            try:
                inst_docker = build_ssh_args(cfg)
                inst_docker.append("curl -fsSL https://get.docker.com | sudo sh 2>/dev/null && sudo usermod -a -G docker $USER 2>/dev/null || true")
                subprocess.run(inst_docker, capture_output=True, text=True, timeout=90)
                chk_d = build_ssh_args(cfg)
                chk_d.append("which docker 2>/dev/null || echo 'NO_DOCKER'")
                res_d = subprocess.run(chk_d, capture_output=True, text=True, timeout=5)
                has_docker = "NO_DOCKER" not in res_d.stdout and "docker" in res_d.stdout
            except Exception:
                pass

        # When Docker is present on Robot PC, container flasher covers esptool and picotool
        if has_docker:
            has_esptool = True
            has_picotool = True

        # Automatic tool installation if missing (zero user action needed)
        auto_installed = False
        if not has_esptool:
            try:
                inst_args = build_ssh_args(cfg)
                inst_args.append("python3 -m pip install -q --user --break-system-packages esptool 2>/dev/null || python3 -m pip install -q --user esptool 2>/dev/null || sudo apt-get install -y -qq python3-esptool 2>/dev/null || true")
                subprocess.run(inst_args, capture_output=True, text=True, timeout=15)
                has_esptool = True
                auto_installed = True
            except Exception:
                pass

        if not has_picotool:
            try:
                inst_pico = build_ssh_args(cfg)
                inst_pico.append("sudo apt-get install -y -qq picotool 2>/dev/null || true; [ -d /etc/udev/rules.d ] && [ ! -f /etc/udev/rules.d/99-pico.rules ] && echo \"ATTRS{idVendor}==\"2e8a\", MODE=\"666\", GROUP=\"dialout\"\" | sudo tee /etc/udev/rules.d/99-pico.rules >/dev/null 2>&1 || true")
                subprocess.run(inst_pico, capture_output=True, text=True, timeout=10)
                has_picotool = True
                auto_installed = True
            except Exception:
                pass

        # Ensure dialout permissions automatically
        try:
            perm_args = build_ssh_args(cfg)
            perm_args.append("sudo usermod -a -G dialout ubuntu 2>/dev/null || true; for _d in /dev/ttyUSB* /dev/ttyACM*; do [ -e \"\" ] && sudo chmod a+rw \"\" 2>/dev/null || true; done")
            subprocess.run(perm_args, capture_output=True, text=True, timeout=5)
        except Exception:
            pass

        os_info = ""
        for line in stdout.split("\n"):
            if "PRETTY_NAME" in line:
                os_info = line.split("=", 1)[-1].strip().strip('"')
                break
        if not os_info:
            for line in stdout.split("\n"):
                if "Linux" in line:
                    os_info = line.strip()
                    break

        return {
            "ok": True,
            "ssh": True,
            "has_esptool": has_esptool,
            "has_picotool": has_picotool,
            "has_docker": has_docker,
            "os_info": os_info or "Linux"
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "SSH connection timed out (5s)"}
    except Exception as e:
        return {"ok": False, "error": str(e)}

def get_remote_serial_ports(cfg):
    args = build_ssh_args(cfg)
    probe_script = (
        "python3 -c '"
        "import glob, json, os\n"
        "patterns = [\"/dev/ttyACM*\", \"/dev/ttyUSB*\", \"/dev/ttyTHS*\", \"/dev/serial0\"]\n"
        "found_ports = []\n"
        "port_details = []\n"
        "for pat in patterns:\n"
        "    for p in glob.glob(pat):\n"
        "        found_ports.append(p)\n"
        "        port_name = os.path.basename(p)\n"
        "        info = {\"port\": p, \"vid\": \"\", \"pid\": \"\", \"product\": \"\", \"manufacturer\": \"\", \"chip\": \"Generic USB Serial\"}\n"
        "        dev_path = f\"/sys/class/tty/{port_name}/device\"\n"
        "        if os.path.exists(dev_path):\n"
        "            curr = os.path.realpath(dev_path)\n"
        "            for _ in range(6):\n"
        "                vp = os.path.join(curr, \"idVendor\")\n"
        "                pp = os.path.join(curr, \"idProduct\")\n"
        "                if os.path.exists(vp) and os.path.exists(pp):\n"
        "                    with open(vp) as f: info[\"vid\"] = f.read().strip().lower()\n"
        "                    with open(pp) as f: info[\"pid\"] = f.read().strip().lower()\n"
        "                    if os.path.exists(os.path.join(curr, \"product\")):\n"
        "                        with open(os.path.join(curr, \"product\")) as f: info[\"product\"] = f.read().strip()\n"
        "                    if os.path.exists(os.path.join(curr, \"manufacturer\")):\n"
        "                        with open(os.path.join(curr, \"manufacturer\")) as f: info[\"manufacturer\"] = f.read().strip()\n"
        "                    break\n"
        "                curr = os.path.dirname(curr)\n"
        "        vid, pid = info[\"vid\"], info[\"pid\"]\n"
        "        if vid == \"10c4\" and pid == \"ea60\":\n"
        "            info[\"chip\"] = \"CP2102N USB Bridge (ESP32/GenDrv)\" if \"cp2102n\" in info[\"product\"].lower() else \"CP2102 USB Bridge (ESP32)\"\n"
        "        elif vid == \"1a86\" and pid in [\"7523\", \"5523\", \"7522\"]:\n"
        "            info[\"chip\"] = \"CH340/CH341 USB Bridge\"\n"
        "        elif vid == \"2e8a\":\n"
        "            info[\"chip\"] = \"Raspberry Pi Pico (RP2040)\" if pid in [\"0003\", \"000a\"] else \"Raspberry Pi Pico 2 (RP2350)\"\n"
        "        elif vid == \"303a\":\n"
        "            info[\"chip\"] = \"ESP32 USB Serial\"\n"
        "        port_details.append(info)\n"
        "print(json.dumps({\"status\": \"ok\", \"serial_ports\": sorted(list(set(found_ports))), \"port_details\": port_details}))\n"
        "'"
    )
    args.append(probe_script)
    try:
        res = subprocess.run(args, capture_output=True, text=True, timeout=8)
        if res.returncode == 0 and res.stdout.strip():
            out = res.stdout.strip()
            idx = out.find('{"status": "ok"')
            if idx >= 0:
                return json.loads(out[idx:])
        # Fallback to simple ls
        fallback_args = build_ssh_args(cfg)
        fallback_args.append("ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
        fres = subprocess.run(fallback_args, capture_output=True, text=True, timeout=5)
        ports = [p.strip() for p in fres.stdout.strip().split("\n") if p.strip()]
        return {"status": "ok", "serial_ports": ports, "port_details": [{"port": p, "chip": "Serial Port"} for p in ports]}
    except Exception as e:
        return {"status": "error", "error": str(e), "serial_ports": [], "port_details": []}

def get_remote_chip_id(cfg, port):
    args = build_ssh_args(cfg)
    script = (
        f"python3 -m esptool --port {port} chip_id 2>&1 || "
        f"~/.platformio/penv/bin/esptool.py --port {port} chip_id 2>&1 || "
        f"esptool.py --port {port} chip_id 2>&1 || "
        f"picotool info 2>&1 || echo 'No chip ID tool available'"
    )
    args.append(script)
    try:
        res = subprocess.run(args, capture_output=True, text=True, timeout=12)
        return {"status": "ok", "output": res.stdout}
    except Exception as e:
        return {"status": "error", "error": str(e)}

def _parse_port_check_output(output, port, mode, udp_port, res):
    fuser_section = ""
    container_section = ""
    process_section = ""

    current_sec = None
    for line in output.splitlines():
        line_s = line.strip()
        if line_s == "---FUSER---":
            current_sec = "fuser"
            continue
        elif line_s == "---CONTAINERS---":
            current_sec = "containers"
            continue
        elif line_s == "---PROCESSES---":
            current_sec = "processes"
            continue

        if current_sec == "fuser":
            fuser_section += line + " "
        elif current_sec == "containers":
            container_section += line + chr(10)
        elif current_sec == "processes":
            process_section += line + chr(10)

    port_basename = os.path.basename(port) if mode != "udp" else str(udp_port)
    for line in container_section.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split("|")
        cid = parts[0]
        cname = parts[1] if len(parts) > 1 else ""
        cimg = parts[2] if len(parts) > 2 else ""
        ccmd = parts[3] if len(parts) > 3 else ""

        is_cont_target = False
        if mode == "udp":
            if f":{udp_port}" in ccmd or f"--port {udp_port}" in ccmd or f"{udp_port}/udp" in line or (("microros" in cimg or "uros" in cname) and "udp" in ccmd):
                is_cont_target = True
        else:
            if port in ccmd or port_basename in ccmd or "microros" in cimg or "uros_agent" in cname or "microros_agent" in cname or "uros" in cname:
                is_cont_target = True

        if is_cont_target:
            res["in_use"] = True
            res["holder_type"] = "container"
            res["container_id"] = cid
            res["container_name"] = cname
            res["is_microros"] = "microros" in cimg or "uros" in cname or "micro_ros" in ccmd
            res["details"] = f"Container '{cname}' ({cid[:12]}): {cimg}"
            res["summary"] = f"Occupied by container '{cname}' ({cid[:8]})"
            return res

    raw_pids = [p for p in fuser_section.replace(":", " ").split() if p.isdigit()]
    if raw_pids:
        res["in_use"] = True
        res["pids"] = raw_pids
        res["holder_type"] = "process"
        if "micro_ros_agent" in process_section:
            res["is_microros"] = True
            res["process_names"] = ["micro_ros_agent"]
            res["summary"] = f"In use by micro_ros_agent (PID {raw_pids[0]})"
        else:
            res["summary"] = f"Port in use by PID {raw_pids[0]}"
        res["details"] = f"PIDs: {raw_pids}"
        return res

    if mode == "udp" and str(udp_port) in process_section:
        res["in_use"] = True
        res["holder_type"] = "socket"
        res["summary"] = f"UDP socket :{udp_port} in use"
        return res

    if mode != "udp" and "micro_ros_agent" in process_section and (port in process_section or port_basename in process_section):
        res["in_use"] = True
        res["holder_type"] = "process"
        res["is_microros"] = True
        res["summary"] = f"micro_ros_agent active on {port}"
        res["details"] = process_section.strip()
        return res

    return res


def check_agent_port_status(port="/dev/ttyUSB0", mode="serial", udp_port=8888, host=None, user=None):
    res = {
        "status": "ok",
        "in_use": False,
        "mode": mode,
        "target": port if mode in ["serial", "multiserial"] else f"UDP:{udp_port}",
        "holder_type": "none",
        "pids": [],
        "process_names": [],
        "container_id": "",
        "container_name": "",
        "is_microros": False,
        "details": "",
        "summary": "Port is available"
    }

    if host:
        ssh_cmd = [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
            "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            f"{user}@{host}" if user else host
        ]
        if mode in ["serial", "multiserial"]:
            script = (
                f"echo '---FUSER---'; fuser '{port}' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"pgrep -fa 'micro_ros_agent' 2>/dev/null || true"
            )
        else:
            script = (
                f"echo '---FUSER---'; fuser '{udp_port}/udp' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"ss -ulnp 'sport = :{udp_port}' 2>/dev/null || true"
            )
        try:
            r = subprocess.run(ssh_cmd + [script], capture_output=True, text=True, timeout=8)
            output = r.stdout
        except Exception as e:
            res["status"] = "error"
            res["details"] = f"SSH error: {e}"
            return res
        return _parse_port_check_output(output, port, mode, udp_port, res)

    # Local port checks
    output_parts = []
    if mode in ["serial", "multiserial"]:
        output_parts.append("---FUSER---")
        if os.path.exists(port):
            try:
                f = subprocess.run(["fuser", port], capture_output=True, text=True, timeout=2)
                output_parts.append(f.stdout)
            except Exception:
                pass
        output_parts.append("---CONTAINERS---")
        try:
            d = subprocess.run(["docker", "ps", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(d.stdout)
        except Exception:
            pass
        try:
            p = subprocess.run(["podman", "ps", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(p.stdout)
        except Exception:
            pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["pgrep", "-fa", "micro_ros_agent"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass
    else:
        output_parts.append("---FUSER---")
        try:
            f = subprocess.run(["fuser", f"{udp_port}/udp"], capture_output=True, text=True, timeout=2)
            output_parts.append(f.stdout)
        except Exception:
            pass
        output_parts.append("---CONTAINERS---")
        try:
            d = subprocess.run(["docker", "ps", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(d.stdout)
        except Exception:
            pass
        try:
            p = subprocess.run(["podman", "ps", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(p.stdout)
        except Exception:
            pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["ss", "-ulnp", f"sport = :{udp_port}"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass

    return _parse_port_check_output("\n".join(output_parts), port, mode, udp_port, res)


def release_agent_port(port="/dev/ttyUSB0", mode="serial", udp_port=8888, host=None, user=None):
    if host:
        ssh_cmd = [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
            "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            f"{user}@{host}" if user else host
        ]
        if mode == "udp":
            script = (
                f"docker stop microros_agent uros_agent_udp 2>/dev/null || true; "
                f"podman stop microros_agent uros_agent_udp 2>/dev/null || true; "
                f"fuser -k -TERM {udp_port}/udp 2>/dev/null || true; "
                f"pkill -f 'micro_ros_agent.*udp' 2>/dev/null || true; "
                f"sleep 0.5"
            )
        else:
            port_base = os.path.basename(port)
            script = (
                f"docker stop microros_agent uros_agent_serial 2>/dev/null || true; "
                f"podman stop microros_agent uros_agent_serial 2>/dev/null || true; "
                f"[ -e '{port}' ] && fuser -k -TERM '{port}' 2>/dev/null || true; "
                f"pkill -f '[m]icro_ros_agent.*{port_base}' 2>/dev/null || true; "
                f"sleep 0.5"
            )
        try:
            r = subprocess.run(ssh_cmd + [script], capture_output=True, text=True, timeout=10)
            return {"status": "ok", "released": True, "output": r.stdout}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    # Local release
    if mode == "udp":
        try:
            subprocess.run(["docker", "stop", "microros_agent", "uros_agent_udp"], capture_output=True, text=True, timeout=5)
        except Exception:
            pass
        try:
            subprocess.run(["podman", "stop", "microros_agent", "uros_agent_udp"], capture_output=True, text=True, timeout=5)
        except Exception:
            pass
        try:
            subprocess.run(["fuser", "-k", "-TERM", f"{udp_port}/udp"], capture_output=True, text=True, timeout=3)
        except Exception:
            pass
        try:
            subprocess.run(["pkill", "-f", "micro_ros_agent.*udp"], capture_output=True, text=True, timeout=3)
        except Exception:
            pass
    else:
        port_base = os.path.basename(port)
        try:
            subprocess.run(["docker", "stop", "microros_agent", "uros_agent_serial"], capture_output=True, text=True, timeout=5)
        except Exception:
            pass
        try:
            subprocess.run(["podman", "stop", "microros_agent", "uros_agent_serial"], capture_output=True, text=True, timeout=5)
        except Exception:
            pass
        if os.path.exists(port):
            try:
                subprocess.run(["fuser", "-k", "-TERM", port], capture_output=True, text=True, timeout=3)
            except Exception:
                pass
        try:
            subprocess.run(["pkill", "-f", f"[m]icro_ros_agent.*{port_base}"], capture_output=True, text=True, timeout=3)
        except Exception:
            pass

    return {"status": "ok", "released": True}


def manage_remote_microros_agent(cfg, action, distro="jazzy", port="/dev/ttyUSB0", baud="1500000"):
    host = cfg.get("robot_host")
    user = cfg.get("robot_user", "ubuntu")

    if action == "status":
        chk = check_agent_port_status(port=port, mode="serial", host=host if host else None, user=user)
        return {"status": "ok", "running": chk["in_use"], "details": chk["details"] or chk["summary"], "check": chk}
    elif action == "start":
        release_agent_port(port=port, mode="serial", host=host if host else None, user=user)
        args = build_ssh_args(cfg)
        script = (
            f"docker run -d --rm --name microros_agent --net=host --privileged "
            f"-v /dev:/dev microros/micro-ros-agent:{distro} "
            f"serial --dev {port} -b {baud}"
        )
        args.append(script)
        res = subprocess.run(args, capture_output=True, text=True, timeout=15)
        return {"status": "ok", "output": res.stdout.strip()}
    elif action == "stop":
        rel = release_agent_port(port=port, mode="serial", host=host if host else None, user=user)
        return {"status": "ok", "stopped": True, "details": rel}
    return {"status": "error", "error": f"Unknown action: {action}"}


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

        if parsed.path == "/api/docker/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(check_container_status()).encode("utf-8"))
            return

        if parsed.path == "/api/workflow/settings":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "settings": load_workflow_settings()}).encode("utf-8"))
            return

        if parsed.path == "/api/docker/rootless_info":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(get_rootless_info()).encode("utf-8"))
            return

        if parsed.path == "/api/docker/setup_rootless":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(setup_rootless_docker()).encode("utf-8"))
            return

        if parsed.path == "/api/container/install":
            qs = parse_qs(parsed.query)
            eng = qs.get("engine", ["docker"])[0]
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(install_container_engine(eng)).encode("utf-8"))
            return

        if parsed.path == "/api/remote/config":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            cfg = load_remote_config()
            self.wfile.write(json.dumps({"status": "ok", "config": cfg}).encode("utf-8"))
            return

        if parsed.path == "/api/remote/serial_ports":
            cfg = load_remote_config()
            qs = parse_qs(parsed.query)
            if "host" in qs: cfg["remote_host"] = qs["host"][0]
            if "user" in qs: cfg["remote_user"] = qs["user"][0]
            if "port" in qs:
                try: cfg["remote_port"] = int(qs["port"][0])
                except: pass
            res = get_remote_serial_ports(cfg)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/remote/chip_id":
            cfg = load_remote_config()
            qs = parse_qs(parsed.query)
            if "host" in qs: cfg["remote_host"] = qs["host"][0]
            if "user" in qs: cfg["remote_user"] = qs["user"][0]
            port = qs.get("port", [cfg.get("remote_serial_port", "/dev/ttyUSB0")])[0]
            res = get_remote_chip_id(cfg, port)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/agent/port_check":
            qs = parse_qs(parsed.query)
            port = qs.get("port", ["/dev/ttyUSB0"])[0]
            mode = qs.get("mode", ["serial"])[0]
            udp_port = int(qs.get("udp_port", [8888])[0])
            host = qs.get("host", [""])[0]
            user = qs.get("user", ["ubuntu"])[0]
            res = check_agent_port_status(port, mode, udp_port, host=host if host else None, user=user)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/remote/microros_agent":
            cfg = load_remote_config()
            res = manage_remote_microros_agent(cfg, "status")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return
        
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
                for distro in ["jazzy", "lyrical", "rolling"]:
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
            # Single-shot frequency measurement. `secs` (default 6, 3..20) is the
            # sampling window — a 1 Hz topic needs several seconds to average
            # meaningfully, so the caller can ask for a longer window on slow
            # topics. The LAST `average rate:` line (most settled) is reported.
            qs = parse_qs(parsed.query)
            topic = qs.get("topic", ["/odom/unfiltered"])[0].strip()
            try:
                secs = int(round(float(qs.get("secs", ["6"])[0])))
            except Exception:
                secs = 6
            secs = max(3, min(secs, 20))
            rate_val = "0.0"
            try:
                setup_bash = ""
                for distro in ["jazzy", "lyrical", "rolling"]:
                    p = f"/opt/ros/{distro}/setup.bash"
                    if os.path.exists(p):
                        setup_bash = p
                        break
                cmd = (f"source {setup_bash} 2>/dev/null && "
                       f"timeout {secs} stdbuf -oL -eL ros2 topic hz --window 10000 {topic}")
                res = subprocess.run(["bash", "-c", cmd], capture_output=True,
                                     text=True, timeout=secs + 5)
                rates = re.findall(r"average rate:\s*([0-9.]+)", res.stdout)
                if rates:
                    rate_val = f"{float(rates[-1]):.2f} Hz"
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
            for distro in ["jazzy", "lyrical", "rolling"]:
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

        if parsed.path == "/api/gitinfo":
            # Git provenance for the header version badge: the 7-char commit the
            # server booted on, plus the live branch / remotes / last 10 commits
            # shown when the badge is clicked.
            info = collect_git_info()
            info["version_at_start"] = GIT_VERSION_AT_START
            info["moved_since_start"] = (
                info.get("version") != GIT_VERSION_AT_START
                and GIT_VERSION_AT_START != "unknown"
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps(info).encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        if parsed.path == "/api/boards":
            # Parse firmware/platformio.ini and return all board environments
            # with their config file path, transport mode, and baud rate.
            ini_path = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
            boards = []
            try:
                boards = _parse_platformio_boards(ini_path)
            except Exception as e:
                pass
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps({"status": "ok", "boards": boards}).encode("utf-8"))
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
                "host_ip": SyslogManager._host_ip(),
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

        if parsed.path == "/api/workflow/settings":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            saved = save_workflow_settings(data)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "settings": saved}).encode("utf-8"))
            return

        if parsed.path == "/api/docker/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(check_container_status()).encode("utf-8"))
            return

        if parsed.path == "/api/docker/rootless_info":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(get_rootless_info()).encode("utf-8"))
            return

        if parsed.path == "/api/docker/setup_rootless":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(setup_rootless_docker()).encode("utf-8"))
            return

        if parsed.path == "/api/container/install":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            eng = data.get("engine", "docker")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(install_container_engine(eng)).encode("utf-8"))
            return

        if parsed.path == "/api/docker/setup_rootless":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(setup_rootless_docker()).encode("utf-8"))
            return

        if parsed.path == "/api/container/install":
            qs = parse_qs(parsed.query)
            eng = qs.get("engine", ["docker"])[0]
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(install_container_engine(eng)).encode("utf-8"))
            return

        if parsed.path == "/api/remote/config":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            saved = save_remote_config(data)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "config": saved}).encode("utf-8"))
            return

        if parsed.path == "/api/docker/install":
            cmd = "curl -fsSL https://get.docker.com | sh 2>/dev/null && sudo usermod -a -G docker $USER 2>/dev/null || true"
            try:
                res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=120)
                ok = shutil.which("docker") is not None
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": ok, "output": res.stdout[-300:]}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": str(e)}).encode("utf-8"))
            return

        if parsed.path == "/api/remote/test":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            cfg = load_remote_config()
            cfg.update(data)
            res = test_remote_ssh(cfg)
            self.send_response(200 if res.get("ok") else 400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/remote/chip_id":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            cfg = load_remote_config()
            cfg.update(data)
            port = data.get("port", cfg.get("remote_serial_port", "/dev/ttyUSB0"))
            res = get_remote_chip_id(cfg, port)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/agent/port_check":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            port = data.get("port", "/dev/ttyUSB0")
            mode = data.get("mode", "serial")
            udp_port = int(data.get("udp_port", 8888))
            host = data.get("host", "")
            user = data.get("user", "ubuntu")
            res = check_agent_port_status(port, mode, udp_port, host=host if host else None, user=user)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/agent/port_release":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            port = data.get("port", "/dev/ttyUSB0")
            mode = data.get("mode", "serial")
            udp_port = int(data.get("udp_port", 8888))
            host = data.get("host", "")
            user = data.get("user", "ubuntu")
            res = release_agent_port(port, mode, udp_port, host=host if host else None, user=user)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

        if parsed.path == "/api/remote/microros_agent":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            cfg = load_remote_config()
            action = data.get("action", "status")
            distro = data.get("distro", "jazzy")
            port = data.get("port", cfg.get("remote_serial_port", "/dev/ttyUSB0"))
            baud = data.get("baud", str(cfg.get("remote_baud", 1500000)))
            res = manage_remote_microros_agent(cfg, action, distro, port, baud)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(res).encode("utf-8"))
            return

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

        if parsed.path == "/api/load_board_config":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8")) if body else {}
            except Exception:
                data = {}
            env_name = data.get("env", "")
            try:
                ini_path = os.path.join(REPO_ROOT, "firmware", "platformio.ini")
                boards = _parse_platformio_boards(ini_path)
                board_info = next((b for b in boards if b["env"] == env_name), None)
                if not board_info:
                    raise ValueError(f"Board environment '{env_name}' not found in platformio.ini")
                config_rel = board_info.get("config_file", "")
                if not config_rel:
                    raise ValueError(f"No config file mapped for environment '{env_name}'")
                config_path = os.path.join(REPO_ROOT, config_rel)
                if not os.path.isfile(config_path):
                    raise FileNotFoundError(f"Config file not found: {config_path}")
                with open(config_path, "r", encoding="utf-8") as hf:
                    header_content = hf.read()
                spec = parse_header_to_spec(header_content)
                # The board header pulls WiFi creds from the git-ignored
                # config/custom/wifi_config.h via __has_include — read them so
                # the form shows the real SSID / password / host IPs.
                merge_wifi_config(spec, _wifi_config_content())
                # Override transport/wifi from platformio.ini flags (authoritative)
                if board_info.get("wifi"):
                    spec["telemetry"]["use_wifi"] = True
                    spec["telemetry"]["transport"] = "WIFI_UDP"
                else:
                    spec["telemetry"]["use_wifi"] = False
                    spec["telemetry"]["transport"] = "SERIAL"
                if board_info.get("monitor_speed"):
                    spec["telemetry"]["baudrate"] = board_info["monitor_speed"]
                # MCU family from platformio.ini platform=/board= (authoritative);
                # env-name map is only a fallback for odd/legacy envs.
                pio_mcu = _mcu_from_platformio(board_info.get("platform", ""),
                                               board_info.get("board", ""), env_name)
                env_to_mcu = {
                    "gendrv": "GENDRV", "gendrv_wifi": "GENDRV",
                    "esp32": "ESP32", "esp32_wifi": "ESP32",
                    "esp32s2": "ESP32S2", "esp32s2_wifi": "ESP32S2",
                    "esp32s3": "ESP32S3", "esp32s3_wifi": "ESP32S3",
                    "pico": "PICO", "picow": "PICOW", "picow_wifi": "PICOW",
                    "pico2": "PICO2", "pico2w": "PICO2W", "pico2w_wifi": "PICO2W",
                    "teensy41": "TEENSY", "teensy40": "TEENSY",
                    "teensy36": "TEENSY", "teensy35": "TEENSY", "teensy31": "TEENSY",
                    "vattenkar": "TEENSY",
                }
                if pio_mcu:
                    spec["mcu"] = pio_mcu
                elif env_name in env_to_mcu:
                    spec["mcu"] = env_to_mcu[env_name]
                # Attach board metadata
                spec["_board_env"] = env_name
                spec["_board_info"] = board_info
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "spec": spec, "board": board_info}).encode("utf-8"))
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
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
                    if '__has_include("wifi_config.h")' in content or "WIFI_AP_LIST" in content:
                        merge_wifi_config(spec, _wifi_config_content())

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

                # WiFi credentials (if any) go to the git-ignored wifi_config.h,
                # never into the tracked robot header. Don't clobber an existing one.
                wifi_config = data.get("wifi_config", "") or ""
                wifi_written = False
                wifi_path = os.path.join(REPO_ROOT, "config", "custom", "wifi_config.h")
                if wifi_config.strip() and not os.path.exists(wifi_path):
                    with open(wifi_path, "w", encoding="utf-8") as wf:
                        wf.write(wifi_config)
                    wifi_written = True

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "saved",
                    "path": os.path.relpath(save_path, REPO_ROOT),
                    "filename": filename,
                    "wifi_config_written": wifi_written
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

            # Single-flight: one build/deploy at a time. Without this a tab that
            # crashed mid-deploy and was reopened could start a second parallel
            # `pio` compile -- ~4 heavy cc1plus jobs on a 4 GB SBC, i.e. OOM.
            with active_process_lock:
                busy = active_process is not None and active_process.poll() is None
            if busy:
                self.send_response(409)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({
                    "error": "A command is already running. Stop it (Cancel) before starting another."
                }).encode("utf-8"))
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
            ros_distro = (data.get("ros_distro") or "").strip()
            if not ros_distro:
                ros_distro = env.get("ROS_DISTRO") or "jazzy"
            env["ROS_DISTRO"] = ros_distro

            client_gone = False

            def send_event(event_type, payload):
                nonlocal client_gone
                msg = f"event: {event_type}\ndata: {json.dumps(payload)}\n\n"
                try:
                    self.wfile.write(msg.encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    client_gone = True

            # Adaptive execution: on a bare host that lacks /opt/ros, route ROS commands into
            # the distrobox named after the requested distro. Never do this when we are already
            # inside a container -- nested `distrobox enter` is unsupported and would hang or fail.
            has_distrobox = shutil.which("distrobox") is not None
            has_host_ros = os.path.exists("/opt/ros")
            is_deploy = ("EOF_DEPLOY_SCRIPT_WRAPPER" in command) or ("deploy_" in command)
            needs_ros = ("ros2" in command or "source /opt/ros" in command) and not is_deploy
            if IN_CONTAINER:
                if needs_ros and not has_host_ros:
                    send_event("output", {
                        "line": f"[INFO] Running inside container '{os.environ.get('CONTAINER_ID', 'container')}'; "
                                f"executing directly (no nested distrobox).\n"
                    })
            elif (not has_host_ros) and has_distrobox and needs_ros:
                try:
                    res = subprocess.run(["distrobox", "list"], capture_output=True, text=True, timeout=2.0)
                    box_exists = ros_distro in res.stdout
                except Exception:
                    box_exists = False
                if box_exists:
                    escaped_cmd = command.replace("\\", "\\\\").replace('"', '\\"')
                    command = (
                        f'distrobox enter {ros_distro} -- bash -c '
                        f'"source /opt/ros/{ros_distro}/setup.bash 2>/dev/null; '
                        f'[ -f $HOME/uros_ws/install/setup.bash ] && source $HOME/uros_ws/install/setup.bash; '
                        f'{escaped_cmd}"'
                    )

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
                        if client_gone:
                            # Browser closed/crashed: kill the whole process
                            # group so a long build doesn't run unwatched.
                            try:
                                os.killpg(os.getpgid(active_process.pid), signal.SIGTERM)
                                active_process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                try:
                                    os.killpg(os.getpgid(active_process.pid), signal.SIGKILL)
                                except Exception:
                                    pass
                            except Exception:
                                pass
                            break

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
