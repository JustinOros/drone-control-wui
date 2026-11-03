import json
import math
import os
import time

from pymavlink import mavutil

HOME_LAT = float(os.environ.get("SIM_LAT", "32.2226"))
HOME_LON = float(os.environ.get("SIM_LON", "-110.9747"))
TARGET = os.environ.get("SIM_OUT", "udpout:127.0.0.1:14550")
BUILDINGS_FILE = os.environ.get(
    "SIM_BUILDINGS", os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_buildings.json")
)
M_PER_DEG = 111320.0
CLIMB = 2.5
DESCEND = 1.0
CRUISE = 5.0
RTL_ALT = 15.0
ACCEPT = 2.0
TICK = 0.05

STABILIZE = 0
ACRO = 1
ALT_HOLD = 2
AUTO = 3
GUIDED = 4
LOITER = 5
RTL = 6
CIRCLE = 7
LAND = 9
KNOWN_MODES = (STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, LOITER, RTL, CIRCLE, LAND)

mav2 = mavutil.mavlink


def point_in_poly(lat, lon, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        yi, xi = pts[i]
        yj, xj = pts[j]
        if (yi > lat) != (yj > lat) and lon < (xj - xi) * (lat - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


class Sim:
    def __init__(self):
        self.conn = mavutil.mavlink_connection(TARGET, source_system=1, source_component=1)
        self.mav = self.conn.mav
        self.boot = time.time()
        self.lat = HOME_LAT
        self.lon = HOME_LON
        self.home = (HOME_LAT, HOME_LON)
        self.alt = 0.0
        self.heading = 0.0
        self.vn = 0.0
        self.ve = 0.0
        self.vu = 0.0
        self.armed = False
        self.mode = STABILIZE
        self.battery = 100.0
        self.items = []
        self.incoming = []
        self.expected = 0
        self.seq = 0
        self.done_said = False
        self.takeoff_alt = None
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.rtl_phase = ""
        self.rtl_alt = RTL_ALT
        self.buildings = []
        self.bmtime = 0.0
        self.last_warn = 0.0

    def load_buildings(self):
        try:
            mtime = os.path.getmtime(BUILDINGS_FILE)
        except OSError:
            self.buildings = []
            self.bmtime = 0.0
            return
        if mtime == self.bmtime:
            return
        self.bmtime = mtime
        try:
            with open(BUILDINGS_FILE) as f:
                raw = json.load(f)
        except (OSError, ValueError):
            return
        out = []
        for item in raw:
            try:
                height = float(item[0])
                flat = item[1]
                pts = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]
            except (TypeError, ValueError, IndexError):
                continue
            if len(pts) < 3:
                continue
            lats = [p[0] for p in pts]
            lons = [p[1] for p in pts]
            out.append((height, pts, min(lats), max(lats), min(lons), max(lons)))
        self.buildings = out
        self.say("{} buildings loaded".format(len(out)))

    def inside_building(self, lat, lon, alt):
        for height, pts, lat_min, lat_max, lon_min, lon_max in self.buildings:
            if alt >= height:
                continue
            if lat < lat_min or lat > lat_max or lon < lon_min or lon > lon_max:
                continue
            if point_in_poly(lat, lon, pts):
                return height
        return None

    def avoid(self, vn, ve, dt):
        if self.inside_building(self.lat, self.lon, self.alt) is not None:
            return vn, ve
        nlat = self.lat + vn * dt / M_PER_DEG
        nlon = self.lon + ve * dt / (M_PER_DEG * math.cos(math.radians(self.lat)))
        if self.inside_building(nlat, nlon, self.alt) is None:
            return vn, ve
        if time.time() - self.last_warn > 3:
            self.last_warn = time.time()
            self.say("Collision: the drone would have hit a building")
        return 0.0, 0.0

    def say(self, text):
        self.mav.statustext_send(mav2.MAV_SEVERITY_INFO, text.encode())

    def ms(self):
        return int((time.time() - self.boot) * 1000)

    def set_mode(self, mode):
        if mode not in KNOWN_MODES:
            return False
        if mode == AUTO and len(self.items) < 2:
            self.say("Mode change failed: no mission")
            return False
        self.mode = mode
        self.takeoff_alt = None
        if mode == AUTO:
            if self.seq < 1 or self.seq >= len(self.items):
                self.seq = 1
            self.send_current()
        if mode == RTL:
            self.rtl_phase = "climb"
            self.rtl_alt = max(self.alt, RTL_ALT)
        return True

    def send_current(self):
        self.mav.mission_current_send(self.seq)

    def advance(self):
        self.seq += 1
        self.send_current()

    def on_command(self, m):
        c = m.command
        result = mav2.MAV_RESULT_ACCEPTED
        if c == mav2.MAV_CMD_COMPONENT_ARM_DISARM:
            if m.param1 == 1:
                if not self.armed:
                    self.armed = True
                    self.home = (self.lat, self.lon)
                    self.say("Arming motors")
            else:
                if self.alt > 0.3:
                    result = mav2.MAV_RESULT_FAILED
                    self.say("Disarm denied: vehicle in air")
                else:
                    self.armed = False
                    self.say("Disarming motors")
        elif c == mav2.MAV_CMD_NAV_TAKEOFF:
            if not self.armed or self.mode != GUIDED:
                result = mav2.MAV_RESULT_FAILED
                self.say("Takeoff denied: arm in GUIDED first")
            else:
                self.takeoff_alt = max(1.0, float(m.param7))
        elif c == mav2.MAV_CMD_DO_SET_MODE:
            if not self.set_mode(int(m.param2)):
                result = mav2.MAV_RESULT_FAILED
        elif c == mav2.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(RTL)
        elif c == mav2.MAV_CMD_NAV_LAND:
            self.set_mode(LAND)
        self.mav.command_ack_send(c, result)

    def on_velocity(self, m):
        self.cmd = (m.vx, m.vy, m.vz, m.yaw_rate, time.time())

    def on_mission_count(self, m):
        self.expected = m.count
        self.incoming = []
        self.items = []
        self.seq = 0
        self.request(m, 0)

    def request(self, m, seq):
        self.mav.mission_request_int_send(m.get_srcSystem(), m.get_srcComponent(), seq)

    def on_mission_item(self, m):
        if m.get_type() == "MISSION_ITEM_INT":
            lat = m.x / 1e7
            lon = m.y / 1e7
        else:
            lat = m.x
            lon = m.y
        if m.seq != len(self.incoming):
            self.request(m, len(self.incoming))
            return
        self.incoming.append((m.command, lat, lon, m.z))
        if len(self.incoming) >= self.expected:
            self.items = self.incoming
            self.mav.mission_ack_send(
                m.get_srcSystem(), m.get_srcComponent(), mav2.MAV_MISSION_ACCEPTED
            )
        else:
            self.request(m, len(self.incoming))

    def on_command_int(self, m):
        if m.command != mav2.MAV_CMD_USER_1:
            return
        result = mav2.MAV_RESULT_ACCEPTED
        if self.armed:
            result = mav2.MAV_RESULT_FAILED
            self.say("Reset denied: disarm first")
        else:
            self.lat = m.x / 1e7
            self.lon = m.y / 1e7
            self.home = (self.lat, self.lon)
            self.alt = 0.0
            self.items = []
            self.incoming = []
            self.seq = 0
            self.say("SIM location set to {:.5f}, {:.5f}".format(self.lat, self.lon))
        self.mav.command_ack_send(m.command, result)

    def handle(self, m):
        t = m.get_type()
        if t == "COMMAND_LONG":
            self.on_command(m)
        elif t == "COMMAND_INT":
            self.on_command_int(m)
        elif t == "SET_MODE":
            self.set_mode(m.custom_mode)
        elif t == "SET_POSITION_TARGET_LOCAL_NED":
            self.on_velocity(m)
        elif t == "MISSION_CLEAR_ALL":
            self.items = []
            self.incoming = []
            self.mav.mission_ack_send(
                m.get_srcSystem(), m.get_srcComponent(), mav2.MAV_MISSION_ACCEPTED
            )
        elif t == "MISSION_COUNT":
            self.on_mission_count(m)
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            self.on_mission_item(m)

    def fly_to(self, lat, lon, alt):
        dn = (lat - self.lat) * M_PER_DEG
        de = (lon - self.lon) * M_PER_DEG * math.cos(math.radians(self.lat))
        dist = math.hypot(dn, de)
        dalt = alt - self.alt
        vn = 0.0
        ve = 0.0
        if dist > 0.01:
            speed = min(CRUISE, max(1.0, dist))
            vn = dn / dist * speed
            ve = de / dist * speed
        vu = max(-2.0, min(CLIMB, dalt))
        yaw_rate = 0.0
        if dist > 3:
            bearing = math.degrees(math.atan2(de, dn)) % 360
            diff = ((bearing - self.heading + 540) % 360) - 180
            yaw_rate = max(-90.0, min(90.0, diff * 2))
        reached = dist < ACCEPT and abs(dalt) < 1.5
        return vn, ve, vu, yaw_rate, reached

    def land(self):
        if self.alt <= 0.05:
            self.alt = 0.0
            self.armed = False
            self.say("Landed, disarming")
            return 0.0, 0.0, 0.0, 0.0
        return 0.0, 0.0, -min(DESCEND, max(0.3, self.alt)), 0.0

    def rtl(self):
        if self.rtl_phase == "climb":
            if self.alt >= self.rtl_alt - 0.3:
                self.rtl_phase = "return"
            else:
                return 0.0, 0.0, CLIMB, 0.0
        if self.rtl_phase == "return":
            vn, ve, vu, yaw_rate, reached = self.fly_to(self.home[0], self.home[1], self.rtl_alt)
            if reached:
                self.rtl_phase = "land"
            return vn, ve, vu, yaw_rate
        return self.land()

    def auto(self):
        if self.seq >= len(self.items):
            if not self.done_said:
                self.done_said = True
                self.say("Mission complete, hovering")
            return 0.0, 0.0, 0.0, 0.0
        self.done_said = False
        command, lat, lon, alt = self.items[self.seq]
        if command == mav2.MAV_CMD_NAV_TAKEOFF:
            if self.alt >= alt - 0.3:
                self.advance()
                return 0.0, 0.0, 0.0, 0.0
            return 0.0, 0.0, CLIMB, 0.0
        if command == mav2.MAV_CMD_NAV_WAYPOINT:
            vn, ve, vu, yaw_rate, reached = self.fly_to(lat, lon, alt)
            if reached:
                self.advance()
            return vn, ve, vu, yaw_rate
        if command == mav2.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(RTL)
            return 0.0, 0.0, 0.0, 0.0
        self.advance()
        return 0.0, 0.0, 0.0, 0.0

    def guided(self):
        if self.takeoff_alt is not None:
            if self.alt >= self.takeoff_alt - 0.3:
                self.takeoff_alt = None
                return 0.0, 0.0, 0.0, 0.0
            return 0.0, 0.0, CLIMB, 0.0
        if time.time() - self.cmd[4] < 1.0:
            fwd, right, down, yaw_rate = self.cmd[:4]
            h = math.radians(self.heading)
            vn = fwd * math.cos(h) - right * math.sin(h)
            ve = fwd * math.sin(h) + right * math.cos(h)
            return vn, ve, -down, math.degrees(yaw_rate)
        return 0.0, 0.0, 0.0, 0.0

    def control(self):
        if self.mode == GUIDED:
            return self.guided()
        if self.mode == LAND:
            return self.land()
        if self.mode == RTL:
            return self.rtl()
        if self.mode == AUTO:
            return self.auto()
        return 0.0, 0.0, 0.0, 0.0

    def step(self, dt):
        vn = ve = vu = yaw_rate = 0.0
        if self.armed:
            vn, ve, vu, yaw_rate = self.control()
            if self.buildings and (vn or ve):
                vn, ve = self.avoid(vn, ve, dt)
            self.battery = max(0.0, self.battery - dt * 100.0 / 1200.0)
        self.heading = (self.heading + yaw_rate * dt) % 360
        self.lat += vn * dt / M_PER_DEG
        self.lon += ve * dt / (M_PER_DEG * math.cos(math.radians(self.lat)))
        self.alt = max(0.0, self.alt + vu * dt)
        self.vn = vn
        self.ve = ve
        self.vu = vu

    def send_heartbeat(self):
        base = mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base |= mav2.MAV_MODE_FLAG_SAFETY_ARMED
        state = mav2.MAV_STATE_ACTIVE if self.armed else mav2.MAV_STATE_STANDBY
        self.mav.heartbeat_send(
            mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA, base, self.mode, state
        )

    def send_position(self):
        self.mav.global_position_int_send(
            self.ms(),
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(self.alt * 1000),
            int(self.alt * 1000),
            int(self.vn * 100),
            int(self.ve * 100),
            int(-self.vu * 100),
            int(self.heading * 100) % 36000,
        )

    def send_hud(self):
        ground = math.hypot(self.vn, self.ve)
        self.mav.vfr_hud_send(
            ground,
            ground,
            int(self.heading) % 360,
            50 if self.armed else 0,
            self.alt,
            self.vu,
        )

    def send_sys(self):
        volts = int(14400 + 2400 * self.battery / 100.0)
        self.mav.sys_status_send(0, 0, 0, 0, volts, 0, int(self.battery), 0, 0, 0, 0, 0, 0)

    def send_gps(self):
        self.mav.gps_raw_int_send(
            int(time.time() * 1e6),
            3,
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(self.alt * 1000),
            80,
            100,
            int(math.hypot(self.vn, self.ve) * 100),
            int(self.heading * 100) % 36000,
            12,
        )

    def run(self):
        print("Bundled simulator running, sending MAVLink to " + TARGET, flush=True)
        last = time.time()
        timers = {"hb": 0.0, "pos": 0.0, "hud": 0.0, "sys": 0.0, "gps": 0.0, "bld": 0.0}
        rates = {"hb": 1.0, "pos": 0.1, "hud": 0.25, "sys": 0.5, "gps": 0.5, "bld": 1.0}
        senders = {
            "hb": self.send_heartbeat,
            "pos": self.send_position,
            "hud": self.send_hud,
            "sys": self.send_sys,
            "gps": self.send_gps,
            "bld": self.load_buildings,
        }
        while True:
            now = time.time()
            dt = now - last
            last = now
            while True:
                m = self.conn.recv_match(blocking=False)
                if m is None:
                    break
                self.handle(m)
            self.step(dt)
            for key, interval in rates.items():
                if now - timers[key] >= interval:
                    timers[key] = now
                    senders[key]()
            time.sleep(TICK)


if __name__ == "__main__":
    try:
        Sim().run()
    except KeyboardInterrupt:
        pass
