# drone-control-wui

A web UI for flying a drone from your computer. Click waypoints on a map, watch the live camera, fly manually with the keyboard or an Xbox controller, and take pictures or record video. Works against an ArduPilot simulator first, then a real drone over WiFi or 4G LTE.

> **Disclaimer:** This app is for testing purposes only. Use it at your own risk. The author is not responsible for any loss of or damage to your drone, property or anything else, or for any injury or legal consequences. Follow your local drone laws and always keep line of sight.

## Features

- Start screen: **Fly with SIM** or **Real flight**
- Click-to-add waypoints on a satellite or street map, one click to launch the mission
- Live drone position, heading, altitude, speed, battery, GPS and flight mode
- Live camera view next to the map, with Popout (own window) and Fullscreen buttons (double-click the video also toggles fullscreen)
- Swap button at the top of the main view: **Show camera** makes the camera the main view and shrinks the map into the camera slot in the side panel (it stays pinned while you scroll, and you can still click it to add waypoints). **Show map** swaps back. The choice is remembered in your browser
- One button to Start, Pause and Play the mission, also on the Xbox Start button. Turning Manual on during a mission pauses it and the button becomes **Resume**, which carries on from the current waypoint instead of restarting the route
- Manual flight: W/S forward and back, A/D turn left and right, Q/E strafe sideways, Space up, Left Shift down
- Xbox (or any standard) controller through the browser Gamepad API, laid out like a standard RC controller: sticks, takeoff, land, return to launch, mission pause and play, speed, photo and video
- Take Picture and Start/Stop Recording, saved to `captures/`
- Takeoff, Land, Return to Launch, Arm, Disarm
- Reset the SIM drone to your real GPS location with one button
- Return home after the last waypoint is on by default. Untick it and the drone hovers at the last waypoint instead (use Land, Return to launch or Manual from there)
- Save, load and delete named routes (waypoints plus altitude, reverse, return home and building settings), stored in `routes.json` next to the server and git-ignored. Load a route, then press Start mission
- Optional reverse leg: after the last waypoint, fly back through the waypoints in reverse, then return to launch
- Metric / Imperial toggle for altitude and speed (remembered in your browser)
- WiFi link for close range, 4G LTE link for long range, selectable at connect time
- Real-world buildings from OpenStreetMap (free), drawn on the map, warned about in mission planning, and solid in the SIM

## How it works

```
Browser (map, video, keyboard, gamepad)
        |  WebSocket + MJPEG
   server.py (FastAPI, pymavlink, OpenCV)
        |  MAVLink
   ArduPilot SITL   or   real flight controller (via companion computer)
```

## Quick start (SIM)

1. Start the web UI. Missing Python packages are installed automatically on first run:

   ```
   python3 server.py
   ```

2. The page opens in your default browser automatically (set `NO_BROWSER=1` to disable). Click **Fly with SIM**.
3. Click the map to add waypoints, then click **Start mission** (or press Start on an Xbox controller). While the mission runs the same button becomes **Pause**, which holds position, and then **Play**, which resumes from the current waypoint.

**Fly with SIM** uses a bundled simulator (`sim.py`) that speaks MAVLink the way ArduCopter does: arm, takeoff, GUIDED velocity control, waypoint missions, RTL and land. Nothing else needs to be installed. It starts at 32.2226, -110.9747 by default, change it with the `SIM_LAT` and `SIM_LON` environment variables.

Click **Reset SIM to my GPS** to move the simulated drone to your real location (the browser asks for location permission). It only works while disarmed, clears any waypoints, and is remembered in `sim_location.json` so the next SIM start begins there. That file is git-ignored.

### Real buildings (free, OpenStreetMap)

Tick **Show real buildings** in the panel and zoom in to level 16 or closer. Building outlines in view are fetched from the OpenStreetMap Overpass API (no account, no cost) and drawn on the map, colored by height.

- Heights come from the `height` or `building:levels` tags. Buildings with neither are assumed to be 9 m.
- The **Building in the way** dropdown picks what happens when your path crosses a building taller than the mission altitude:
  - **Fly around** (default) adds detour waypoints around the buildings, including the way back to the launch point, and previews the new route as a dashed blue line. Start mission sends the detoured route to the drone, so it works for real flights too.
  - **Fly over** raises the mission altitude to clear the tallest building on the path, plus a 5 m margin.
  - **Warn only** leaves the route alone and shows a warning with the tallest building and the altitude to raise it to.
- If no way around is found (for example a waypoint sits inside a building), a warning shows and Start mission asks you to confirm.
- Detours keep about 8 m of clearance and are only as good as the OpenStreetMap data, so always keep line of sight and watch the drone.
- The SIM camera draws the buildings as simple 3D boxes (walls and roofs) from the drone's position and heading.
- Manual flight has a building guard (works in SIM and real flights, while the buildings checkbox is on). The page stops forward and sideways movement when a building taller than your altitude is about 6 m plus 1.5 seconds of travel ahead, and shows a red BUILDING AHEAD, STOPPED banner. Climb, turn and back away still work. It only knows OpenStreetMap footprints and heights, and GPS is not exact, so always keep line of sight.
- In the SIM the buildings are also solid as a last resort. If the drone ever touches one it stops with "Collision: the drone would have hit a building", which on a real drone would be a crash. The SIM does not plan a path around buildings.
- Terrain is treated as flat, and only building footprints are used (no trees, wires or terrain).
- Data is OpenStreetMap contributors, licensed ODbL. The attribution shows on the map when buildings are on. Please respect the Overpass fair-use limits. Loaded areas are cached and requests are debounced.
- Loaded areas are cached in your browser (about 1 km tiles, kept for 30 days), so reloading the page or restarting the server does not fetch them again. Click **Clear cache** next to the buildings checkbox, or start the server with `python3 server.py --clear-cache`, to forget them and reload fresh data.
- The footprints are handed to the simulator through `sim_buildings.json`, which is git-ignored and recreated on each start.

If a real ArduPilot SITL is already sending MAVLink to UDP port 14550, the SIM button connects to that instead and the bundled simulator is not started:

```
sim_vehicle.py -v ArduCopter --map --console
```

The SIM camera is a synthetic view so the video panel, picture and recording buttons can all be tested without hardware.

## Real flight

Click **Real flight**, pick a link, fill in the connection details, and confirm the safety prompt.

| Link | Range | MAVLink connection | Video |
|------|-------|--------------------|-------|
| WiFi | 100 to 300 m, 1 to 2 km with a directional antenna | `udpin:0.0.0.0:14550` (drone companion computer forwards to your computer's IP) | RTSP or MJPEG URL from the drone |
| 4G LTE | Anywhere with cell coverage | `tcp:your-relay-host:5760` (a relay server running mavlink-router) | RTSP or MJPEG URL via the relay |

On the drone, a Raspberry Pi (or similar) reads the flight controller serial port and forwards MAVLink, for example with `mavlink-router` or MAVProxy, and streams the camera with MediaMTX or GStreamer. For LTE, the Pi and your computer both connect out to a relay on a server with a public address, so no port forwarding at home is needed.

Settings are remembered in your browser.

### Safety

- Steering (forward, back, strafe and yaw) is locked while the altitude reads under 0.5 m, in the page and in the server. You can still climb from the ground.
- Return to launch goes back to the spot where the drone was last armed, so if you relaunch from a new spot, that spot becomes home.
- Configure return-to-launch on link loss, low battery and GPS loss in ArduPilot before the first flight.
- Test everything in SIM first, then with props off, then tethered or low and close.
- WASD over 4G adds latency (about 50 to 150 ms or more). Use waypoints for long range and keep manual control for close range.
- Follow your local drone laws. In the US that means FAA registration over 250 g, visual line of sight, and a 400 ft altitude limit unless you hold a waiver. The server caps altitude at 120 m.
- The server binds to 127.0.0.1 by default. It has no login, so do not expose it to the internet.

## Compatible drones you can just buy

This project talks MAVLink to an ArduPilot flight controller. Any drone running ArduPilot (ArduCopter is the tested target) should work. If you would rather buy than build, these are good starting points. Prices are rough and change often, so check current listings.

| Drone | Flight stack | Rough price | Notes |
|-------|--------------|-------------|-------|
| Holybro X500 V2 kit (Pixhawk 6C) | PX4 or ArduPilot | $500 to $1,000 | Popular 500 mm dev frame, GPS included, easy to add a Pi, camera and LTE modem |
| Holybro S500 V2 kit | PX4 or ArduPilot | $400 to $700 | Cheaper 500 mm dev frame, same idea |
| Holybro QAV250 kit | PX4 or ArduPilot | $300 to $600 | Smaller, faster 250 mm frame |
| ModalAI Starling 2 | PX4 | $3,000 and up | Ready-to-fly with onboard computer and cameras, LTE add-on modems available, pricey but the least assembly |
| Skywalker X8 airframe plus Pixhawk-class controller | ArduPlane | $300 to $700 | Fixed-wing for long endurance and range, a classic LTE long-range platform |

Notes:

- ArduPilot is the primary target. PX4 uses different flight mode names, so some buttons (Land, RTL, Manual mode) may need adjusting.
- Most kits need a companion computer (Raspberry Pi Zero 2 W or Pi 4), a camera, and for long range a 4G LTE modem with a data SIM. Video over LTE uses roughly 1 to 3 GB per hour at 720p.
- **Not compatible:** DJI, Autel, Skydio consumer drones, Parrot ANAFI, and toy drones. They use closed or proprietary control protocols, not MAVLink.

## Build your own

A typical build is about $250 to $450:

- Flight controller running ArduPilot (Matek H743 or Pixhawk class)
- GPS and compass module, ESCs, motors, 5 to 7 inch frame, LiPo or Li-ion battery
- Raspberry Pi Zero 2 W or Pi 4 as companion computer
- Pi camera or USB camera
- 4G LTE modem for long range

## Controls

| Input | Action |
|-------|--------|
| W / S | Forward / back |
| A / D | Turn (yaw) left / right |
| Q / E | Strafe left / right |
| Space | Up |
| Left Shift | Down |
| Xbox left stick up / down | Climb / descend |
| Xbox left stick left / right | Yaw |
| Xbox right stick | Move forward, back, left, right |
| Xbox Y | Arm and take off to the alt box height, then switch to Manual |
| Xbox X | Land |
| Xbox Back / View | Return to launch |
| Xbox Start | Start, pause or resume the mission. Also the stop button: during Land, Return to launch, a takeoff climb or a mission start it halts the action and holds position |
| Xbox L3 (press left stick) | Manual on or off |
| Xbox LB / RB | Speed down / up |
| Xbox A | Take picture |
| Xbox B | Start or stop recording |

Click the gear icon in the top right of the panel to remap any controller button or stick, switch between Mode 1 and Mode 2 stick layouts, invert an axis, or reset to the defaults above. Mappings are saved in your browser.

After a halt the drone hovers and pressing Start again does nothing until you take control (Manual, Takeoff, Land or RTL), so a second press cannot restart something by accident. The on-screen mission button works the same way and shows Hold while an action can be halted.

The default Xbox layout follows a standard RC controller (Mode 2): left stick is throttle and yaw, right stick is pitch and roll. Centered sticks hold position and altitude.

Turn on **Manual** in the panel first. Manual control switches the drone to GUIDED mode.

## Roadmap

- Test against full ArduPilot SITL before every release
- WebRTC video for lower latency
- Automatic WiFi to LTE link failover
- Saved missions and geofence drawing
- PX4 mode support

## License

MIT

> **Disclaimer:** This app is for testing purposes only. Use it at your own risk. The author is not responsible for any loss of or damage to your drone, property or anything else, or for any injury or legal consequences. Follow your local drone laws and always keep line of sight.
