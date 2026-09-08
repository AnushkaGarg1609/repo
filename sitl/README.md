# Software-In-The-Loop Harness

Flies the real firmware against a simulated vehicle and talks to Mission
Planner or QGroundControl over UDP. No board required.

## Build and run

```
cd sitl
make
./sitl_bridge          # Windows: sitl_bridge.exe
```

Optional arguments: `./sitl_bridge <gcs-host> <gcs-port> <local-port>`
(defaults `127.0.0.1 14550 14551`).

## Connecting a ground station

**Mission Planner** — set the connection dropdown to `UDP`, port `14550`,
press **Connect**. It listens; the harness transmits to it and adopts whatever
source address the reply comes from.

**QGroundControl** — connects automatically on its default UDP listener.

Then: draw a mission on the map and **Write**, **Arm**, **Takeoff**, and set
mode **AUTO** (custom mode 4) to fly it.

## Flight modes as the GCS sees them

`custom_mode` carries this firmware's own `flight_mode_t`, and HEARTBEAT sets
`CUSTOM_MODE_ENABLED` so the ground station reads it verbatim:

| custom_mode | Mode |
|---|---|
| 0 | DISARMED |
| 1 | STABILIZE |
| 2 | ALT_HOLD |
| 3 | POS_HOLD |
| 4 | MISSION |
| 5 | RTL |

The vehicle advertises `MAV_AUTOPILOT_GENERIC`, so Mission Planner shows the
raw number rather than mislabelling it with ArduPilot's mode names.

## Making Mission Planner offer its full UI (compat build)

Because the firmware honestly reports `GENERIC`, Mission Planner hides the
copter-specific controls: the **Takeoff** button and the flight-mode dropdown
are absent, and it pops a "can't update home" warning every few seconds. All
of these are UI gates keyed to the autopilot type, not firmware faults — you
can still fly by sending the same commands through **Actions -> Do Action**.

If you would rather have the point-and-click UI, build the compat binary:

```
make compat
./sitl_bridge_ardupilot          # Windows: sitl_bridge_ardupilot.exe
```

It is the same firmware with one preprocessor flag (`-DMAV_ARDUPILOT_COMPAT`).
Two advertised fields change, and nothing else:

- HEARTBEAT reports `MAV_AUTOPILOT_ARDUPILOTMEGA`, so Mission Planner loads its
  ArduCopter UI — Takeoff button, mode dropdown, home controls all appear.
- `custom_mode` is remapped from this firmware's `flight_mode_t` onto
  ArduCopter's mode numbers, so each mode shows the right name:

  | this firmware | ArduCopter (name shown) |
  |---|---|
  | STABILIZE | 0 STABILIZE |
  | ALT_HOLD | 2 ALT_HOLD |
  | POS_HOLD | 16 POSHOLD |
  | MISSION | 3 AUTO |
  | RTL | 6 RTL |

- `MAV_CMD_DO_SET_HOME` is accepted-and-ignored (the firmware keeps the home it
  latched at arming, per Section 2.5), which silences the warning.

**The compat build is for ground-station testing only. The real board build
never defines the macro** — it tells the truth about what it is. The
`gcs_probe.py` check expects the honest mode numbers and so is run against the
default binary, not this one.

## Automated check

`gcs_probe.py` is a ground station written directly against the MAVLink v2
specification, deliberately sharing no code with `mavlink/`. Agreement between
the two is the evidence that the wire format is right — reusing the C structs
would only prove they are self-consistent.

```
./sitl_bridge &          # in one terminal
python gcs_probe.py      # in another
```

It runs connect → read parameters → upload mission → download and verify →
arm → take off → fly in AUTO → confirm each waypoint → disarm, and reports 33
checks. Requires only the Python standard library.

## What is real and what is simulated

| Real firmware (identical to the Aries V3 build) | Simulated (PC only) |
|---|---|
| `core/` — Q16.16 math, CORDIC, complementary filters, cascaded PID, mixer, mode machine, arming, mission sequencer | `core/vehicle_sim.c` — 6-DOF rigid-body physics |
| `mavlink/` — encoder, parser, ground-control bridge | `sitl_link.c` — UDP standing in for a SiK radio |
| | the RC block in `sitl_bridge.c` — a transmitter that is not here |

Nothing in `core/` or `mavlink/` is stubbed or compiled differently. The fixed-
point arithmetic is bit-exact with what the RV32IM core will execute.

`sitl_link.c` is the only file in the project that talks to an operating
system. On hardware it is replaced by two Taurus SDK calls:

```
sitl_link_recv()  ->  taurus_uart_read_byte(UART_GCS)
sitl_link_send()  ->  taurus_uart_write(UART_GCS, ...)
sitl_millis()     ->  the superloop tick counter (Section 3.4)
```

## Known simulation artefacts

These are properties of the simulation, not of the firmware.

- **Disarming at altitude** drops the vehicle. The altitude estimate overshoots
  negative during the fall and takes a few seconds to reconverge once the
  complementary filter reweights toward the barometer. Not a normal operation.
- **`apply_ground_contact()` in `sitl_bridge.c`** is a landing-gear model that
  `vehicle_sim.c` does not have. Without it a hard landing leaves the vehicle
  spinning on its side indefinitely, so it can never satisfy the Section 11.1
  "gyro stationary" arming check and the session is stuck until restarted.
## Home position

Home is latched twice, and this is deliberate:

1. **Provisionally, at the first good GPS fix** — a ground station uploads its
   mission before arming, and waypoints in degE7 have no local frame to
   convert into until an origin exists.
2. **For real, at arming** — as Section 2.5 specifies.

The second latch is safe because `mavlink_bridge_set_home()` re-projects any
loaded mission from the coordinates the ground station originally sent. The
bridge keeps those alongside the NED offsets for exactly this reason.

Without that, moving home would silently translate the whole mission by
however far home moved: power on beside the car, walk 400 m to the launch
site, and every waypoint would land 400 m past where it was drawn, with no
error reported anywhere. `test_home_relatch()` in `mavlink/test_mavlink.c`
covers the case.

**On hardware, the GPS driver must convert its fixes against this same
origin.** Call `mavlink_bridge_get_home()` rather than keeping a second copy —
two definitions of home that drift apart put the mission frame and the
estimator frame in different places, and the vehicle flies a consistent,
confident, wrong distance from every waypoint.
