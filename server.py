import asyncio
import atexit
import importlib.util
import json
import math
import os
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

REQUIRED = {
    "fastapi": "fastapi",
    "uvicorn": "uvicorn[standard]",
    "websockets": "websockets",
    "pymavlink": "pymavlink",
    "cv2": "opencv-python-headless",
    "numpy": "numpy",
}


def ensure_deps():
    missing = []
    for module, package in REQUIRED.items():
        if importlib.util.find_spec(module) is None and package not in missing:
            missing.append(package)
    if not missing:
        return
    print("Installing missing dependencies: " + ", ".join(missing), flush=True)
    base = [sys.executable, "-m", "pip", "install", "--quiet"]
    try:
        subprocess.check_call(base + missing)
    except subprocess.CalledProcessError:
        try:
            subprocess.check_call(base + ["--break-system-packages"] + missing)
        except subprocess.CalledProcessError:
            print("Automatic install failed. Run: pip install -r requirements.txt", flush=True)
            sys.exit(1)
    importlib.invalidate_caches()


ensure_deps()

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

BASE = Path(__file__).parent
CAPTURES = BASE / "captures"
CAPTURES.mkdir(exist_ok=True)
SIM_CONNECTION = "udpin:0.0.0.0:14550"
FONT = cv2.FONT_HERSHEY_SIMPLEX
MAX_ALT = 120.0
MAX_SPEED = 18.0
MAX_YAW = 90.0
GROUND_ALT = 0.5
prefs = {"imperial": False}


def stamp():
    return time.strftime("%Y%m%d_%I%M%S%p")


def clamp(value, low, high):
    return max(low, min(high, float(value)))


class Vehicle:
    def __init__(self):
        self.lock = threading.RLock()
        self.link = None
        self.gen = 0
        self.mode_gen = 0
        self.state = self.blank()

    @staticmethod
    def blank():
        return {
            "connected": False,
            "armed": False,
            "mode": "",
            "lat": None,
            "lon": None,
            "alt": 0.0,
            "heading": 0,
            "groundspeed": 0.0,
            "battery": None,
            "voltage": None,
            "sats": 0,
            "fix": 0,
            "wp": 0,
            "msg": "",
            "msg_id": 0,
        }

    def snapshot(self):
        return dict(self.state)

    def need(self):
        if self.link is None:
            raise RuntimeError("not connected")

    def connect(self, conn, timeout=20):
        self.disconnect()
        link = mavutil.mavlink_connection(conn, autoreconnect=True, source_system=255)
        hb = link.wait_heartbeat(timeout=timeout)
        if hb is None:
            link.close()
            raise RuntimeError("no heartbeat from vehicle on " + conn)
        link.mav.request_data_stream_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )
        self.gen += 1
        self.link = link
        self.state = self.blank()
        self.state["connected"] = True
        threading.Thread(target=self.loop, args=(self.gen,), daemon=True).start()

    def disconnect(self):
        self.gen += 1
        with self.lock:
            if self.link is not None:
                try:
                    self.link.close()
                except Exception:
                    pass
            self.link = None
        self.state = self.blank()

    def loop(self, gen):
        while self.gen == gen:
            msg = None
            with self.lock:
                if self.link is not None:
                    try:
                        msg = self.link.recv_match(blocking=False)
                    except Exception:
                        msg = None
            if msg is None:
                time.sleep(0.01)
                continue
            self.handle(msg)

    def handle(self, msg):
        kind = msg.get_type()
        s = self.state
        if kind == "HEARTBEAT":
            if msg.get_srcSystem() != self.link.target_system:
                return
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            s["armed"] = bool(self.link.motors_armed())
            s["mode"] = self.link.flightmode
        elif kind == "GLOBAL_POSITION_INT":
            s["lat"] = msg.lat / 1e7
            s["lon"] = msg.lon / 1e7
            s["alt"] = msg.relative_alt / 1000.0
            if msg.hdg != 65535:
                s["heading"] = int(msg.hdg / 100)
        elif kind == "VFR_HUD":
            s["groundspeed"] = round(msg.groundspeed, 1)
        elif kind == "SYS_STATUS":
            s["voltage"] = round(msg.voltage_battery / 1000.0, 2)
            s["battery"] = msg.battery_remaining if msg.battery_remaining >= 0 else None
        elif kind == "GPS_RAW_INT":
            s["sats"] = msg.satellites_visible
            s["fix"] = msg.fix_type
        elif kind == "MISSION_CURRENT":
            s["wp"] = msg.seq
        elif kind == "STATUSTEXT":
            s["msg"] = msg.text
            s["msg_id"] += 1
            prefix = "SIM location set to "
            if msg.text.startswith(prefix):
                try:
                    lat, lon = msg.text[len(prefix):].split(",")
                    save_location(float(lat), float(lon))
                except (ValueError, OSError):
                    pass

    def command(self, cmd, *params):
        self.need()
        padded = list(params) + [0] * (7 - len(params))
        with self.lock:
            self.link.mav.command_long_send(
                self.link.target_system,
                self.link.target_component,
                cmd,
                0,
                *padded,
            )

    def set_mode(self, name):
        self.need()
        mapping = self.link.mode_mapping()
        if name not in mapping:
            raise RuntimeError("unknown mode " + name)
        with self.lock:
            self.link.set_mode(mapping[name])

    def set_location(self, lat, lon):
        self.need()
        with self.lock:
            self.link.mav.command_int_send(
                self.link.target_system,
                self.link.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_USER_1,
                0, 0,
                0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), 0,
            )

    def arm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def wait_armed(self, timeout=20):
        end = time.time() + timeout
        while time.time() < end:
            if self.state["armed"]:
                return True
            time.sleep(0.2)
        return False

    def arm_and_wait(self):
        if self.state["armed"]:
            return
        self.arm()
        if not self.wait_armed():
            raise RuntimeError("arm failed: " + (self.state["msg"] or "no reason given"))

    def takeoff(self, alt):
        alt = clamp(alt, 2, MAX_ALT)
        self.set_mode("GUIDED")
        time.sleep(0.5)
        self.arm_and_wait()
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)

    def control(self, fwd, right, up, yaw):
        if self.link is None or self.state["mode"] != "GUIDED":
            return
        fwd = clamp(fwd, -MAX_SPEED, MAX_SPEED)
        right = clamp(right, -MAX_SPEED, MAX_SPEED)
        up = clamp(up, -MAX_SPEED / 2, MAX_SPEED / 2)
        yaw = clamp(yaw, -MAX_YAW, MAX_YAW)
        if self.state["alt"] < GROUND_ALT:
            fwd = 0.0
            right = 0.0
            yaw = 0.0
            up = max(0.0, up)
        with self.lock:
            self.link.mav.set_position_target_local_ned_send(
                0,
                self.link.target_system,
                self.link.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                1479,
                0, 0, 0,
                fwd, right, -up,
                0, 0, 0,
                0, math.radians(yaw),
            )

    def upload_mission(self, wps, alt, rth=True, takeoff=True):
        self.need()
        alt = clamp(alt, 2, MAX_ALT)
        home_lat = self.state["lat"] if self.state["lat"] is not None else wps[0][0]
        home_lon = self.state["lon"] if self.state["lon"] is not None else wps[0][1]
        items = [(0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home_lat, home_lon, 0.0)]
        if takeoff:
            items.append((6, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, alt))
        for lat, lon in wps:
            items.append((6, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt))
        if rth:
            items.append((6, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0))
        ts = self.link.target_system
        tc = self.link.target_component
        with self.lock:
            mav = self.link.mav
            mav.mission_clear_all_send(ts, tc)
            time.sleep(0.5)
            while self.link.recv_match(blocking=False) is not None:
                pass
            mav.mission_count_send(ts, tc, len(items))
            deadline = time.time() + 25
            while time.time() < deadline:
                msg = self.link.recv_match(
                    type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                    blocking=True,
                    timeout=1,
                )
                if msg is None:
                    continue
                if msg.get_type() == "MISSION_ACK":
                    if msg.type == 0:
                        return
                    raise RuntimeError("mission rejected, code " + str(msg.type))
                frame, cmd, lat, lon, z = items[msg.seq]
                mav.mission_item_int_send(
                    ts, tc, msg.seq, frame, cmd, 0, 1,
                    0, 0, 0, 0,
                    int(lat * 1e7), int(lon * 1e7), z,
                )
        raise RuntimeError("mission upload timed out")

    def update_mission(self, wps, alt, rth=True):
        if not wps:
            raise RuntimeError("no waypoints")
        self.set_mode("LOITER")
        time.sleep(0.4)
        self.upload_mission(wps, alt, rth, takeoff=False)
        ts = self.link.target_system
        tc = self.link.target_component
        with self.lock:
            self.link.mav.mission_set_current_send(ts, tc, 1)
        time.sleep(0.3)
        self.set_mode("AUTO")

    def run_mission(self, wps, alt, rth=True):
        if not wps:
            raise RuntimeError("no waypoints")
        gen = self.mode_gen
        self.upload_mission(wps, alt, rth)
        self.set_mode("GUIDED")
        time.sleep(0.5)
        self.arm_and_wait()
        if self.mode_gen != gen:
            raise RuntimeError("mission start cancelled, flight mode was changed")
        self.set_mode("AUTO")


_bld_cache = {"mtime": None, "data": []}


def sim_buildings():
    try:
        m = BUILDINGS_FILE.stat().st_mtime
    except OSError:
        _bld_cache["mtime"] = None
        _bld_cache["data"] = []
        return _bld_cache["data"]
    if m != _bld_cache["mtime"]:
        try:
            _bld_cache["data"] = json.loads(BUILDINGS_FILE.read_text())
            _bld_cache["mtime"] = m
        except (OSError, ValueError):
            pass
    return _bld_cache["data"]


def draw_buildings(img, st, horizon):
    lat0, lon0 = st.get("lat"), st.get("lon")
    if lat0 is None or lon0 is None:
        return
    data = sim_buildings()
    if not data:
        return
    alt = st.get("alt") or 0.0
    hdg = math.radians(st.get("heading") or 0)
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    sh, ch = math.sin(hdg), math.cos(hdg)
    focal = 320.0
    near = 0.5
    sky = (205, 170, 100)
    items = []
    for h, flat in data:
        pts = []
        for i in range(0, len(flat) - 1, 2):
            n = (flat[i] - lat0) * mlat
            e = (flat[i + 1] - lon0) * mlon
            pts.append((n * ch + e * sh, -n * sh + e * ch))
        if len(pts) < 3:
            continue
        near_d = min(math.hypot(f, r) for f, r in pts)
        if near_d > 500:
            continue
        area = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
        items.append((near_d, h, pts, area))
    items.sort(key=lambda it: -it[0])

    def px(x, y):
        return [int(max(-5000, min(5000, x))), int(max(-5000, min(5000, y)))]

    for dist0, h, pts, area in items:
        base = (150, 150, 165) if h > 15 else (140, 160, 190)
        fog = min(1.0, dist0 / 450.0) * 0.5

        def tint(shade):
            return tuple(int(c * shade * (1 - fog) + bg * fog) for c, bg in zip(base, sky))

        walls = []
        for i in range(len(pts)):
            a = pts[i]
            b = pts[(i + 1) % len(pts)]
            ex, ey = b[0] - a[0], b[1] - a[1]
            cross = ex * (0 - a[1]) - ey * (0 - a[0])
            if cross * area > 0:
                continue
            if a[0] < near and b[0] < near:
                continue
            if a[0] < near:
                t = (near - a[0]) / (b[0] - a[0])
                a = (near, a[1] + t * (b[1] - a[1]))
            elif b[0] < near:
                t = (near - b[0]) / (a[0] - b[0])
                b = (near, b[1] + t * (a[1] - b[1]))
            shade = 0.55 + 0.45 * abs(b[1] - a[1]) / (math.hypot(b[1] - a[1], b[0] - a[0]) + 1e-6)
            walls.append(((a[0] + b[0]) / 2, shade, a, b))
        walls.sort(key=lambda w: -w[0])
        for _, shade, a, b in walls:
            q = [
                px(320 + focal * a[1] / a[0], horizon - focal * (h - alt) / a[0]),
                px(320 + focal * b[1] / b[0], horizon - focal * (h - alt) / b[0]),
                px(320 + focal * b[1] / b[0], horizon + focal * alt / b[0]),
                px(320 + focal * a[1] / a[0], horizon + focal * alt / a[0]),
            ]
            poly = np.array(q, np.int32)
            cv2.fillPoly(img, [poly], tint(shade))
            cv2.polylines(img, [poly], True, (40, 40, 50), 1)
        if alt > h and all(f >= near for f, r in pts):
            roof = np.array([px(320 + focal * r / f, horizon - focal * (h - alt) / f) for f, r in pts], np.int32)
            cv2.fillPoly(img, [roof], tint(1.0))
            cv2.polylines(img, [roof], True, (40, 40, 50), 1)


def sim_frame(t, st):
    img = np.zeros((360, 640, 3), np.uint8)
    horizon = int(180 + 20 * math.sin(t / 2))
    img[:horizon] = (205, 170, 100)
    img[horizon:] = (70, 130, 70)
    cv2.line(img, (0, horizon), (640, horizon), (255, 255, 255), 1)
    draw_buildings(img, st, horizon)
    cv2.putText(img, "SIM CAMERA", (16, 32), FONT, 0.8, (255, 255, 255), 2)
    cv2.putText(img, time.strftime("%I:%M:%S %p"), (500, 32), FONT, 0.6, (255, 255, 255), 1)
    alt = st.get("alt") or 0.0
    spd = st.get("groundspeed") or 0.0
    hdg = st.get("heading") or 0
    if prefs["imperial"]:
        line = "ALT {:.0f} ft   SPD {:.1f} mph   HDG {}".format(alt * 3.28084, spd * 2.23694, hdg)
    else:
        line = "ALT {:.1f} m   SPD {:.1f} m/s   HDG {}".format(alt, spd, hdg)
    cv2.putText(img, line, (16, 340), FONT, 0.6, (255, 255, 255), 1)
    return img


def no_signal_frame():
    img = np.zeros((360, 640, 3), np.uint8)
    cv2.putText(img, "NO VIDEO", (230, 190), FONT, 1.2, (200, 200, 200), 2)
    return img


class Video:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.gen = 0
        self.writer = None
        self.size = None
        self.recording = False

    def start(self, source, sim, state_fn):
        self.stop()
        self.gen += 1
        threading.Thread(target=self.run, args=(self.gen, source, sim, state_fn), daemon=True).start()

    def stop(self):
        self.gen += 1
        self.stop_recording()
        with self.lock:
            self.frame = None

    def run(self, gen, source, sim, state_fn):
        cap = None
        while self.gen == gen:
            if sim:
                frame = sim_frame(time.time(), state_fn())
                time.sleep(1 / 15)
            else:
                ok = False
                frame = None
                if cap is None and source:
                    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
                if cap is not None:
                    ok, frame = cap.read()
                if not ok:
                    if cap is not None:
                        cap.release()
                        cap = None
                    frame = no_signal_frame()
                    time.sleep(1)
            self.push(frame)
        if cap is not None:
            cap.release()

    def push(self, frame):
        with self.lock:
            self.frame = frame
            if self.recording and self.writer is not None:
                out = frame
                if (frame.shape[1], frame.shape[0]) != self.size:
                    out = cv2.resize(frame, self.size)
                self.writer.write(out)

    def get(self):
        with self.lock:
            return self.frame

    def photo(self):
        with self.lock:
            if self.frame is None:
                return ""
            name = "photo_" + stamp() + ".jpg"
            cv2.imwrite(str(CAPTURES / name), self.frame)
            return name

    def start_recording(self):
        with self.lock:
            if self.recording or self.frame is None:
                return ""
            h, w = self.frame.shape[:2]
            name = "video_" + stamp() + ".mp4"
            self.writer = cv2.VideoWriter(
                str(CAPTURES / name), cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h)
            )
            self.size = (w, h)
            self.recording = True
            return name

    def stop_recording(self):
        with self.lock:
            if self.writer is not None:
                self.writer.release()
            self.writer = None
            self.recording = False


BUILDINGS_FILE = BASE / "sim_buildings.json"
BUILDINGS_FILE.unlink(missing_ok=True)
LOCATION_FILE = BASE / "sim_location.json"


def load_location():
    try:
        data = json.loads(LOCATION_FILE.read_text())
        return float(data["lat"]), float(data["lon"])
    except Exception:
        return None


def save_location(lat, lon):
    LOCATION_FILE.write_text(json.dumps({"lat": lat, "lon": lon}))


class SimProcess:
    def __init__(self):
        self.proc = None

    def start(self):
        self.stop()
        env = dict(os.environ)
        loc = load_location()
        if loc is not None:
            env["SIM_LAT"] = str(loc[0])
            env["SIM_LON"] = str(loc[1])
        self.proc = subprocess.Popen([sys.executable, str(BASE / "sim.py")], env=env)

    def stop(self):
        if self.proc is None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None


vehicle = Vehicle()
video = Video()
simproc = SimProcess()
atexit.register(simproc.stop)
session = {"mode": ""}
app = FastAPI()
app.mount("/captures", StaticFiles(directory=CAPTURES), name="captures")


@app.get("/")
def index():
    return FileResponse(BASE / "index.html")


CLEAR_CACHE = "--clear-cache" in sys.argv
STARTUP_ID = str(time.time())


@app.get("/cacheflag")
def cacheflag():
    return {"clear": CLEAR_CACHE, "id": STARTUP_ID}


ROUTES_FILE = BASE / "routes.json"
routes_lock = threading.Lock()


def read_routes():
    try:
        data = json.loads(ROUTES_FILE.read_text())
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def write_routes(data):
    tmp = ROUTES_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=1))
    os.replace(tmp, ROUTES_FILE)


@app.get("/routes")
def list_routes():
    with routes_lock:
        return {"routes": sorted(read_routes().keys(), key=str.lower)}


@app.get("/routes/{name}")
def get_route(name: str):
    with routes_lock:
        data = read_routes()
    if name not in data:
        raise HTTPException(status_code=404, detail="no such route")
    return data[name]


@app.post("/routes")
async def save_route(request: Request):
    body = await request.json()
    name = str(body.get("name", "")).strip()[:60]
    if not name:
        raise HTTPException(status_code=400, detail="name required")
    try:
        wps = [[float(p[0]), float(p[1])] for p in body.get("waypoints", [])][:200]
    except (TypeError, ValueError, IndexError):
        raise HTTPException(status_code=400, detail="bad waypoints")
    if not wps:
        raise HTTPException(status_code=400, detail="no waypoints")
    route = {
        "waypoints": wps,
        "alt": clamp(float(body.get("alt", 20)), 2, MAX_ALT),
        "reverse": bool(body.get("reverse", False)),
        "rth": bool(body.get("rth", True)),
        "avoid": body.get("avoid") if body.get("avoid") in ("around", "over", "warn") else "around",
    }
    with routes_lock:
        data = read_routes()
        existed = name in data
        data[name] = route
        write_routes(data)
    return {"ok": True, "name": name, "updated": existed}


@app.delete("/routes/{name}")
def delete_route(name: str):
    with routes_lock:
        data = read_routes()
        if name not in data:
            raise HTTPException(status_code=404, detail="no such route")
        del data[name]
        write_routes(data)
    return {"ok": True}


@app.get("/popout")
def popout():
    return FileResponse(BASE / "popout.html")


@app.post("/buildings")
async def buildings(request: Request):
    data = await request.json()
    if not isinstance(data, list):
        raise HTTPException(status_code=400, detail="expected a list")
    data = data[:4000]
    tmp = BUILDINGS_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(data))
    os.replace(tmp, BUILDINGS_FILE)
    return {"ok": True, "count": len(data)}


def mjpeg():
    while True:
        frame = video.get()
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
        time.sleep(0.06)


@app.get("/video")
def video_stream():
    return StreamingResponse(mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")


async def handle(msg):
    cmd = msg.get("cmd")
    if cmd == "connect":
        mode = msg.get("mode")
        if mode == "sim":
            source = ""
            try:
                await asyncio.to_thread(vehicle.connect, SIM_CONNECTION, 3)
            except RuntimeError:
                await asyncio.to_thread(simproc.start)
                await asyncio.to_thread(vehicle.connect, SIM_CONNECTION, 20)
        else:
            simproc.stop()
            conn = (msg.get("conn") or "").strip()
            source = (msg.get("video") or "").strip()
            if not conn:
                raise RuntimeError("connection string required")
            await asyncio.to_thread(vehicle.connect, conn)
        video.start(source, mode == "sim", vehicle.snapshot)
        session["mode"] = mode
        return {"type": "connected", "mode": mode}
    if cmd == "disconnect":
        video.stop()
        vehicle.disconnect()
        simproc.stop()
        session["mode"] = ""
        return {"type": "disconnected"}
    if cmd == "control":
        vehicle.control(
            msg.get("fwd", 0), msg.get("right", 0), msg.get("up", 0), msg.get("yaw", 0)
        )
        return None
    if cmd == "units":
        prefs["imperial"] = bool(msg.get("imperial"))
        return None
    if cmd == "simloc":
        if session["mode"] != "sim":
            raise RuntimeError("location reset is only available in SIM mode")
        lat = float(msg.get("lat"))
        lon = float(msg.get("lon"))
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise RuntimeError("invalid coordinates")
        await asyncio.to_thread(vehicle.set_location, lat, lon)
        return {"type": "ok", "message": "location sent to SIM"}
    if cmd == "arm":
        await asyncio.to_thread(vehicle.arm)
        return {"type": "ok", "message": "arm sent"}
    if cmd == "disarm":
        vehicle.mode_gen += 1
        await asyncio.to_thread(vehicle.disarm)
        return {"type": "ok", "message": "disarm sent"}
    if cmd == "takeoff":
        await asyncio.to_thread(vehicle.takeoff, msg.get("alt", 10))
        return {"type": "ok", "message": "takeoff sent"}
    if cmd == "land":
        vehicle.mode_gen += 1
        await asyncio.to_thread(vehicle.set_mode, "LAND")
        return {"type": "ok", "message": "landing"}
    if cmd == "rtl":
        vehicle.mode_gen += 1
        await asyncio.to_thread(vehicle.set_mode, "RTL")
        return {"type": "ok", "message": "returning to launch"}
    if cmd == "mode":
        vehicle.mode_gen += 1
        await asyncio.to_thread(vehicle.set_mode, msg.get("mode", "GUIDED"))
        return {"type": "ok", "message": "mode " + msg.get("mode", "GUIDED")}
    if cmd == "mission":
        wps = [(float(p[0]), float(p[1])) for p in msg.get("waypoints", [])]
        full = wps
        if msg.get("reverse") and len(wps) > 1:
            full = wps + wps[-2::-1]
        rth = bool(msg.get("rth", True))
        await asyncio.to_thread(vehicle.run_mission, full, msg.get("alt", 20), rth)
        text = "mission started with {} waypoints".format(len(wps))
        if len(full) > len(wps):
            text += " plus the reverse leg"
        if not rth:
            text += ", it will hover at the last waypoint"
        return {"type": "ok", "message": text}
    if cmd == "mission_update":
        wps = [(float(p[0]), float(p[1])) for p in msg.get("waypoints", [])]
        rth = bool(msg.get("rth", True))
        vehicle.mode_gen += 1
        await asyncio.to_thread(vehicle.update_mission, wps, msg.get("alt", 20), rth)
        return {"type": "ok", "message": "mission updated, {} waypoints left".format(len(wps))}
    if cmd == "photo":
        name = video.photo()
        if not name:
            raise RuntimeError("no video frame available")
        return {"type": "capture", "file": "/captures/" + name}
    if cmd == "record":
        if msg.get("on"):
            name = video.start_recording()
            if not name:
                raise RuntimeError("could not start recording")
            return {"type": "ok", "message": "recording " + name}
        video.stop_recording()
        return {"type": "ok", "message": "recording stopped"}
    raise RuntimeError("unknown command " + str(cmd))


@app.websocket("/ws")
async def ws(socket: WebSocket):
    await socket.accept()

    async def pusher():
        while True:
            await socket.send_text(
                json.dumps(
                    {
                        "type": "telemetry",
                        "data": vehicle.snapshot(),
                        "recording": video.recording,
                        "session": session,
                    }
                )
            )
            await asyncio.sleep(0.2)

    task = asyncio.create_task(pusher())
    try:
        while True:
            raw = await socket.receive_text()
            try:
                reply = await handle(json.loads(raw))
            except Exception as exc:
                reply = {"type": "error", "message": str(exc)}
            if reply:
                await socket.send_text(json.dumps(reply))
    except WebSocketDisconnect:
        pass
    finally:
        task.cancel()


if __name__ == "__main__":
    import uvicorn

    host = os.environ.get("HOST", "127.0.0.1")
    port = int(os.environ.get("PORT", "8000"))
    if os.environ.get("NO_BROWSER") != "1":
        url = "http://{}:{}".format("127.0.0.1" if host == "0.0.0.0" else host, port)
        threading.Timer(1.5, webbrowser.open, args=(url,)).start()
    uvicorn.run(app, host=host, port=port)
