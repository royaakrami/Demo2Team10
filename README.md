# Demo 2 Project
# Team 10


# MEETING LOG LINK:

https://mines0-my.sharepoint.com/:w:/g/personal/roya_akrami_mines_edu/IQD3sN2tfWRoQawaOW5fZ3i1AU7zyexo1oPlGrZNvjgGSHI?e=7hrLKB


# Purpose
The robot turns until the camera finds the beacon (ArUco marker 0, 6x6), drives up to it and stops about 1.25 ft away.
For the type 2 trials it then tuns 90 degrees toward the arrow: green arrow on the left = turn left, red arrow on the right = turn right.

# How to Run
1. Pi: run `bash pi/install_autostart.sh` once. After that the vision code starts by itself every time the Pi boots.
2. Turn on the Pi first and wait for the LCD to say `Marker not seen` and `WAIT`.
3. Arduino: upload `arduino/Demo2_Beacon/Demo2_Beacon.ino`. Put a jumper from A3 to GND for type 2 and leave it open for type 1. Then press reset or turn on the battery. The timer starts here.

# For Pi
Pi uses the USB cam to find marker 0 on every frame.
It works out the angle to the marker (left positive, right negative), the distance, and which arrows next to it.
It sends all of that to the Arduino over I2C on every frame, even when the marker isn't seen.
It also shows the distance, angle, arrow and the Arduino's state on the LCD.

We don't use the checkerboard calibration anymore bc it was a failure. Instead, `quick_focal.py` measures the focal length with a tape measure (we got 666 px).

# Arduino
This uses the same encoder and feedback control as Demo 1.
WAIT: it waits for the Pi. SEARCH: it turns 35 degrees, stops to look, and repeats until it sees the marker.
ALIGN: it turns to face the marker. APPROACH: it drives in and keeps correcting with every camera frame. The last 0.5 ft is done on the encoders only.
FINAL_TURN but only tyoe 2: it turns 90 degrees toward the arrow. DONE: it prints the result on Serial.

# Pi Files:

**beacon_vision.py** is the main program that runs on the robot. Camera, then detection, then I2C. It shows a preview window when a monitor is attached and saves every frame to `logs/`.

**vision_settings.py** has settings for camera, exposure, focal length, marker size, arrow colors, LCD.

**beacon_detect.py** finds marker, angle and distance (ie solvePnP), and arrow color.

**quick_focal.py** measures focal length w/ a tape measure and saves `camera_focal.json`.

**arduino_link.py** builds and sends the I2C packet (wiring layout in `I2C_PROTOCOL.md`).

**fake_vision.py** tests the Arduino over I2C without the camera. You type fake marker fixes in.

**install_autostart.sh** makes `beacon_vision.py` start at boot, because the demo is untethered.

**tests/** hs offline tests that need no camera or Arduino: `bash pi/tests/run_all.sh`

# Arduino Files Explained:

**Demo2_Beacon.ino** is  state machine, the vision handling and the control loop.

**Demo2Config.h** has Demo 2 settings: stop distance, camera offset, search step, trial type, speed limits.

**Demo2Types.h** has states and packet struct.

**RobotConfig.h** and **RobotIO.h** are the same as Demo 1.

# Before the demo
- The camera sits over the weel axle, so `CAM_FWD_OFFSET_FT = 0.0f` in `Demo2Config.h`. If the camera moves forward again, put the gap there in ft (10 cm = 0.328f), or the robot stops too far away.
- `STOP_DIST_FT = 1.25f` is measured from the axle. the rule is within 1.5 ft.
- The black square on the printed beacon has to be 2.00 in. If it isn't, put the real size in `MARKER_SIZE_M` in `vision_settings.py`.
- `camera_focal.json` (666 px) is in the `pi` folder. If you run `quick_focal.py` again, it overwrites it.
- After moving the camera, check that the marker straight ahead at 36 in reads about 3.0 ft and about 0 degrees.

# How to test it

**Camera only:** run `python3 beacon_vision.py --dry-run`.
- Marker straight ahead at 36 in: about 0 degrees and 3.0 ft.
- 12 in to the left: about +18 degrees. 12 in to the right: about -18 degrees.
- Left beacon shows `arrow LEFT`, right beacon shows `arrow RIGHT`.
- At 6 ft the marker should still be seen most of the time, at 15 fps or more.

What we got: 3.03 ft at 36 in, +19.0 and -19.6 degrees at the side spots, 6.5 ft at 72 in, both arrows right.

**Focal length:** run `python3 quick_focal.py`. Put the beacon at 24 in and then 36 in, and type `y` to save.

# PROJECT BOARD RESPONSIBILITIES:

Gerardo & Roya : Pi Code/Testing
Basil & Mohammed : Arduino Code/Testing
