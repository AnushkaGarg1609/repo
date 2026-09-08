#!/usr/bin/env python3
"""
gcs_probe.py --- a minimal ground station, written against the MAVLink v2
specification independently of the C code it is testing.

The point of writing the framing again here, rather than reusing anything from
mavlink/, is that agreement between the two is evidence. If the C encoder and
this decoder --- built from the spec by different routes --- produce and accept
the same bytes, the wire format is right. Reusing the C structs would only
prove they are self-consistent.

It runs a full scripted session against sitl_bridge:

    connect -> read parameters -> upload a mission -> arm -> take off
            -> fly the mission in AUTO -> confirm each waypoint -> disarm

and reports PASS/FAIL per step.

Usage:  python gcs_probe.py [--port 14550] [--timeout 120]
"""

import argparse
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
# MAVLink v2 framing, implemented from the specification
# ---------------------------------------------------------------------------

STX_V2 = 0xFD

# msg_id -> (crc_extra, struct format in wire order, field names)
MESSAGES = {
    0:   (50,  "<IBBBBB", "custom_mode type autopilot base_mode system_status mavlink_version"),
    1:   (124, "<IIIHHhHHHHHHb",
          "sensors_present sensors_enabled sensors_health load voltage_battery "
          "current_battery drop_rate_comm errors_comm errors_count1 errors_count2 "
          "errors_count3 errors_count4 battery_remaining"),
    21:  (159, "<BB", "target_system target_component"),
    22:  (220, "<fHH16sB", "param_value param_count param_index param_id param_type"),
    24:  (24,  "<QiiiHHHHBB",
          "time_usec lat lon alt eph epv vel cog fix_type satellites_visible"),
    30:  (39,  "<Iffffff",
          "time_boot_ms roll pitch yaw rollspeed pitchspeed yawspeed"),
    33:  (104, "<IiiiihhhH",
          "time_boot_ms lat lon alt relative_alt vx vy vz hdg"),
    42:  (28,  "<H", "seq"),
    43:  (132, "<BB", "target_system target_component"),
    44:  (221, "<HBB", "count target_system target_component"),
    46:  (11,  "<H", "seq"),
    47:  (153, "<BBB", "target_system target_component type"),
    51:  (196, "<HBB", "seq target_system target_component"),
    73:  (38,  "<ffffiifHHBBBBB",
          "param1 param2 param3 param4 x y z seq command "
          "target_system target_component frame current autocontinue"),
    74:  (20,  "<ffffhH", "airspeed groundspeed alt climb heading throttle"),
    76:  (152, "<fffffffHBBB",
          "param1 param2 param3 param4 param5 param6 param7 command "
          "target_system target_component confirmation"),
    77:  (143, "<HB", "command result"),
    253: (83,  "<B50s", "severity text"),
}

NAMES = {
    0: "HEARTBEAT", 1: "SYS_STATUS", 21: "PARAM_REQUEST_LIST", 22: "PARAM_VALUE",
    24: "GPS_RAW_INT", 30: "ATTITUDE", 33: "GLOBAL_POSITION_INT",
    42: "MISSION_CURRENT", 43: "MISSION_REQUEST_LIST", 44: "MISSION_COUNT",
    46: "MISSION_ITEM_REACHED", 47: "MISSION_ACK", 51: "MISSION_REQUEST_INT",
    73: "MISSION_ITEM_INT", 74: "VFR_HUD", 76: "COMMAND_LONG",
    77: "COMMAND_ACK", 253: "STATUSTEXT",
}


def crc_accumulate(crc, byte):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc_compute(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        crc = crc_accumulate(crc, b)
    return crc_accumulate(crc, crc_extra)


class Msg:
    def __init__(self, msg_id, fields, sysid, compid, seq):
        self.id = msg_id
        self.name = NAMES.get(msg_id, str(msg_id))
        self.sysid = sysid
        self.compid = compid
        self.seq = seq
        self.__dict__.update(fields)

    def __repr__(self):
        return "%s(%s)" % (self.name, self.__dict__)


class Codec:
    """Encodes and decodes MAVLink v2, including the payload-truncation rule."""

    def __init__(self, sysid=255, compid=190):
        self.sysid = sysid
        self.compid = compid
        self.seq = 0
        self.buf = bytearray()
        self.stats = {"frames": 0, "crc_errors": 0, "unknown": 0}

    def encode(self, msg_id, **fields):
        crc_extra, fmt, names = MESSAGES[msg_id]
        values = [fields.get(n, 0) for n in names.split()]
        # bytes fields (param_id, text) need padding to their declared width
        values = [v.encode() if isinstance(v, str) else v for v in values]
        payload = bytearray(struct.pack(fmt, *values))

        # v2 truncation: drop trailing zero bytes, keeping at least one
        while len(payload) > 1 and payload[-1] == 0:
            payload.pop()

        header = bytearray()
        header.append(len(payload))
        header.append(0)            # incompat_flags
        header.append(0)            # compat_flags
        header.append(self.seq)
        header.append(self.sysid)
        header.append(self.compid)
        header += bytes([msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF])
        self.seq = (self.seq + 1) & 0xFF

        crc = crc_compute(header + payload, crc_extra)
        return bytes([STX_V2]) + bytes(header) + bytes(payload) + struct.pack("<H", crc)

    def feed(self, data):
        """Push received bytes; yield decoded messages."""
        self.buf += data
        out = []
        while True:
            start = self.buf.find(bytes([STX_V2]))
            if start < 0:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 12:
                break
            payload_len = self.buf[1]
            frame_len = 10 + payload_len + 2
            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            payload = frame[10:10 + payload_len]
            crc_rx = struct.unpack("<H", frame[frame_len - 2:frame_len])[0]

            if msg_id not in MESSAGES:
                self.stats["unknown"] += 1
                del self.buf[:1]
                continue

            crc_extra, fmt, names = MESSAGES[msg_id]
            if crc_compute(frame[1:10 + payload_len], crc_extra) != crc_rx:
                self.stats["crc_errors"] += 1
                del self.buf[:1]        # resynchronise
                continue

            # zero-pad back to the declared length before unpacking
            full = struct.calcsize(fmt)
            padded = payload + b"\x00" * (full - len(payload))
            values = struct.unpack(fmt, padded[:full])
            fields = dict(zip(names.split(), values))
            out.append(Msg(msg_id, fields, frame[5], frame[6], frame[4]))
            self.stats["frames"] += 1
            del self.buf[:frame_len]
        return out


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------

results = []


def step(ok, label, detail=""):
    results.append((ok, label))
    mark = "[ OK ]" if ok else "[FAIL]"
    print("  %s %s%s" % (mark, label, ("   " + detail) if detail else ""))
    sys.stdout.flush()


class Session:
    def __init__(self, port, verbose=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.codec = Codec()
        self.peer = None
        self.verbose = verbose
        self.latest = {}         # msg name -> most recent message
        self.events = []         # ordered log of interesting messages
        self.last_hb = 0.0

    def pump(self, duration=0.0):
        """Service the link for `duration` seconds, collecting messages."""
        end = time.time() + duration
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                self.peer = addr
                for m in self.codec.feed(data):
                    self.latest[m.name] = m
                    if m.id in (22, 44, 46, 47, 51, 73, 77, 253):
                        self.events.append(m)
                    if self.verbose and m.id in (47, 77, 253):
                        print("        <- %r" % m)
            except BlockingIOError:
                pass
            except OSError:
                pass

            now = time.time()
            if now - self.last_hb > 1.0 and self.peer:
                # A ground station heartbeat; the vehicle uses it for the
                # Section 11.2 telemetry-link timer.
                self.send(0, type=6, autopilot=8, base_mode=0,
                          custom_mode=0, system_status=4, mavlink_version=3)
                self.last_hb = now

            if now >= end:
                return
            time.sleep(0.002)

    def send(self, msg_id, **fields):
        if not self.peer:
            return
        self.sock.sendto(self.codec.encode(msg_id, **fields), self.peer)

    def wait_for(self, name, timeout, predicate=None):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            m = self.latest.get(name)
            if m and (predicate is None or predicate(m)):
                return m
        return None

    def take_event(self, msg_id, timeout, predicate=None):
        """Wait for a message that arrives as a discrete event, not a stream."""
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            for m in list(self.events):
                if m.id == msg_id and (predicate is None or predicate(m)):
                    self.events.remove(m)
                    return m
        return None

    def command(self, cmd, p1=0.0, p2=0.0, p7=0.0, timeout=3.0):
        self.events = [e for e in self.events if e.id != 77]
        self.send(76, command=cmd, param1=p1, param2=p2, param7=p7,
                  target_system=1, target_component=1, confirmation=0)
        return self.take_event(77, timeout, lambda m: m.command == cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    print("=" * 68)
    print(" MAVLink ground-station probe --- independent spec implementation")
    print("=" * 68)

    s = Session(args.port, args.verbose)
    t_start = time.time()

    # -- 1. Connect ---------------------------------------------------------
    print("\n=== Connection ===")
    hb = s.wait_for("HEARTBEAT", 15.0)
    step(hb is not None, "HEARTBEAT received from the vehicle")
    if hb is None:
        print("\n  No heartbeat. Is sitl_bridge running?")
        return 1
    step(hb.type == 2, "vehicle identifies as MAV_TYPE_QUADROTOR", "type=%d" % hb.type)
    step(hb.mavlink_version == 3, "reports MAVLink v2", "version=%d" % hb.mavlink_version)
    step((hb.base_mode & 0x01) != 0, "CUSTOM_MODE_ENABLED is set so custom_mode is readable")
    step((hb.base_mode & 0x80) == 0, "reports disarmed at startup")
    step(s.codec.stats["crc_errors"] == 0, "no CRC errors on the inbound stream")

    step(s.wait_for("SYS_STATUS", 3.0) is not None, "SYS_STATUS arrives")
    sysst = s.latest.get("SYS_STATUS")
    if sysst:
        step(11000 < sysst.voltage_battery < 12000,
             "battery voltage is at the expected wire offset",
             "%.2f V" % (sysst.voltage_battery / 1000.0))
        step(sysst.battery_remaining == 85,
             "battery_remaining survives the six error counters",
             "%d %%" % sysst.battery_remaining)

    step(s.wait_for("ATTITUDE", 2.0) is not None, "ATTITUDE stream arrives")
    step(s.wait_for("GLOBAL_POSITION_INT", 2.0) is not None, "GLOBAL_POSITION_INT stream arrives")
    step(s.wait_for("VFR_HUD", 2.0) is not None, "VFR_HUD stream arrives")

    gps = s.wait_for("GPS_RAW_INT", 3.0)
    step(gps is not None and gps.fix_type >= 3, "GPS reports a 3D fix",
         "fix=%d sats=%d" % (gps.fix_type, gps.satellites_visible) if gps else "")

    pos = s.latest.get("GLOBAL_POSITION_INT")
    home_lat, home_lon = 0, 0
    if pos:
        home_lat, home_lon = pos.lat, pos.lon
        step(abs(pos.lat / 1e7 - 12.9716) < 0.01 and abs(pos.lon / 1e7 - 77.5946) < 0.01,
             "position decodes to the expected coordinates",
             "%.5f, %.5f" % (pos.lat / 1e7, pos.lon / 1e7))

    # -- 2. Parameters ------------------------------------------------------
    print("\n=== Parameter protocol ===")
    s.events = [e for e in s.events if e.id != 22]
    s.send(21, target_system=1, target_component=1)
    s.pump(2.0)
    params = {}
    for m in s.events:
        if m.id == 22:
            params[m.param_id.rstrip(b"\x00").decode()] = (m.param_value, m.param_count)
    step(len(params) >= 10, "the parameter list streams out", "%d parameters" % len(params))
    step("RATE_RLL_P" in params, "RATE_RLL_P is present")
    if "RATE_RLL_P" in params:
        step(abs(params["RATE_RLL_P"][0] - 60.0) < 0.01,
             "RATE_RLL_P carries the live gain", "%.3f" % params["RATE_RLL_P"][0])

    # -- 3. Mission upload --------------------------------------------------
    print("\n=== Mission upload ===")
    # Four waypoints in a square, expressed the way a ground station would:
    # absolute degE7 with altitude relative to home.
    M_PER_DEG = 111194.9266
    legs = [(0.0, 0.0), (15.0, 0.0), (15.0, 15.0), (0.0, 0.0)]
    alt = 15.0
    import math
    coslat = math.cos(math.radians(home_lat / 1e7))
    wps = []
    for (n, e) in legs:
        wps.append((int(home_lat + n / M_PER_DEG * 1e7),
                    int(home_lon + e / (M_PER_DEG * coslat) * 1e7),
                    alt))

    s.events = [e for e in s.events if e.id not in (44, 47, 51)]
    s.send(44, count=len(wps), target_system=1, target_component=1)

    uploaded = 0
    ack = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        req = s.take_event(51, 2.0)
        if req is None:
            ack = s.take_event(47, 0.5)
            break
        i = req.seq
        if i >= len(wps):
            break
        lat, lon, a = wps[i]
        s.send(73, seq=i, command=16, frame=6, current=0, autocontinue=1,
               param1=0.0, param2=3.0, param3=0.0, param4=0.0,
               x=lat, y=lon, z=a, target_system=1, target_component=1)
        uploaded += 1
        ack = s.take_event(47, 0.5)
        if ack is not None:
            break

    step(uploaded == len(wps), "every waypoint was requested in order",
         "%d of %d" % (uploaded, len(wps)))
    step(ack is not None and ack.type == 0, "MISSION_ACK reports ACCEPTED",
         "type=%s" % (ack.type if ack else "none"))

    # -- 4. Mission download (read it back) ---------------------------------
    print("\n=== Mission download ===")
    s.events = [e for e in s.events if e.id != 44]
    s.send(43, target_system=1, target_component=1)
    cnt = s.take_event(44, 3.0)
    step(cnt is not None and cnt.count == len(wps),
         "the vehicle reports back the same waypoint count",
         "count=%s" % (cnt.count if cnt else "none"))

    round_trip_ok = True
    max_err = 0.0
    s.events = [e for e in s.events if e.id != 73]
    for i in range(len(wps)):
        s.send(51, seq=i, target_system=1, target_component=1)
        item = s.take_event(73, 3.0, lambda m, i=i: m.seq == i)
        if item is None:
            round_trip_ok = False
            break
        dn = (item.x - wps[i][0]) / 1e7 * M_PER_DEG
        de = (item.y - wps[i][1]) / 1e7 * M_PER_DEG * coslat
        max_err = max(max_err, abs(dn), abs(de), abs(item.z - wps[i][2]))
    s.send(47, target_system=1, target_component=1, type=0)
    step(round_trip_ok and max_err < 0.5,
         "waypoints survive the upload/download round trip",
         "worst error %.2f m" % max_err)

    # -- 5. Arm ------------------------------------------------------------
    print("\n=== Arming ===")
    ack = s.command(400, p1=1.0)          # MAV_CMD_COMPONENT_ARM_DISARM
    step(ack is not None, "arm command is acknowledged")
    step(ack is not None and ack.result == 0, "arm is ACCEPTED",
         "result=%s" % (ack.result if ack else "none"))

    hb = s.wait_for("HEARTBEAT", 3.0, lambda m: (m.base_mode & 0x80) != 0)
    step(hb is not None, "the heartbeat now reports ARMED")

    # -- 6. Takeoff --------------------------------------------------------
    print("\n=== Takeoff ===")
    ack = s.command(22, p7=alt)           # MAV_CMD_NAV_TAKEOFF
    step(ack is not None and ack.result == 0, "takeoff command is ACCEPTED")

    climbed = False
    end = time.time() + 40.0
    while time.time() < end:
        s.pump(0.2)
        hud = s.latest.get("VFR_HUD")
        if hud and hud.alt > alt * 0.85:
            climbed = True
            break
    hud = s.latest.get("VFR_HUD")
    step(climbed, "the vehicle climbs to the commanded altitude",
         "%.1f m of %.1f m" % (hud.alt if hud else 0.0, alt))

    # -- 7. Fly the mission ------------------------------------------------
    print("\n=== Autonomous mission ===")
    ack = s.command(300)                  # MAV_CMD_MISSION_START
    step(ack is not None and ack.result == 0, "MISSION_START is ACCEPTED")

    hb = s.wait_for("HEARTBEAT", 5.0, lambda m: m.custom_mode == 4)   # MODE_MISSION
    step(hb is not None, "the vehicle enters MODE_MISSION",
         "custom_mode=%s" % (s.latest["HEARTBEAT"].custom_mode))

    reached = set()
    max_wp = 0
    end = time.time() + max(20.0, args.timeout - (time.time() - t_start) - 10.0)
    while time.time() < end:
        s.pump(0.2)
        for m in list(s.events):
            if m.id == 46:
                reached.add(m.seq)
                s.events.remove(m)
                print("        waypoint %d reached" % m.seq)
        mc = s.latest.get("MISSION_CURRENT")
        if mc:
            max_wp = max(max_wp, mc.seq)
        hb = s.latest.get("HEARTBEAT")
        if hb and hb.custom_mode == 5:      # MODE_RTL --- mission finished
            break

    step(max_wp >= 1, "the sequencer advances through the mission",
         "reached waypoint index %d" % max_wp)
    step(len(reached) >= 1, "MISSION_ITEM_REACHED is emitted",
         "%d notifications" % len(reached))

    pos = s.latest.get("GLOBAL_POSITION_INT")
    if pos:
        dn = (pos.lat - home_lat) / 1e7 * M_PER_DEG
        de = (pos.lon - home_lon) / 1e7 * M_PER_DEG * coslat
        print("        final position: N %+.1f m, E %+.1f m, alt %.1f m"
              % (dn, de, pos.relative_alt / 1000.0))
        step(abs(dn) < 60 and abs(de) < 60,
             "the vehicle stayed within the mission area")

    # -- 8. Disarm ---------------------------------------------------------
    print("\n=== Disarm ===")
    ack = s.command(400, p1=0.0)
    step(ack is not None and ack.result == 0, "disarm is ACCEPTED")
    hb = s.wait_for("HEARTBEAT", 3.0, lambda m: (m.base_mode & 0x80) == 0)
    step(hb is not None, "the heartbeat reports disarmed again")

    # -- Summary -----------------------------------------------------------
    print("\n" + "=" * 68)
    print(" link: %d frames decoded, %d CRC errors, %d unknown message IDs"
          % (s.codec.stats["frames"], s.codec.stats["crc_errors"],
             s.codec.stats["unknown"]))
    passed = sum(1 for ok, _ in results if ok)
    print(" %d checks run, %d failed" % (len(results), len(results) - passed))
    if passed == len(results):
        print(" ALL CHECKS PASSED")
    else:
        print(" *** FAILURES ***")
        for ok, label in results:
            if not ok:
                print("   - " + label)
    print("=" * 68)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
