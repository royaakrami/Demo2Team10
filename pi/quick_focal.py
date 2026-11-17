#!/usr/bin/env python3
# this measures the camera focal length with a tape measure, so we dont need the checkerboard
# put the beacon straight ahead, measure from the front of the lens to the paper in inches,
# type it in and dont touch anything for about 2 s. do 24 and 36 in, enter on an empty line, then y to save
# it saves camera_focal.json and beacon_vision.py uses that automatically

import json
import statistics
import time
from pathlib import Path

import numpy as np

import vision_settings as S
from beacon_detect import BeaconFinder, load_focal
from beacon_vision import FrameGrabber

HERE = Path(__file__).resolve().parent


def read_marker(cam, finder, frames=30, timeout=6.0):
    # median distance, angle and marker size over a few frames, None if it wasnt seen
    d, a, px = [], [], []
    count, start = 0, time.monotonic()
    # skip the first frames because they can be from before you let go of the marker
    while time.monotonic() - start < 0.4:
        got = cam.next(count)
        if got:
            count = got[1]
    start = time.monotonic()
    while len(d) < frames and time.monotonic() - start < timeout:
        got = cam.next(count)
        if got is None:
            continue
        frame, count, _ = got
        fix = finder.process(frame)
        if fix.seen:
            d.append(fix.dist_m)
            a.append(fix.angle_deg)
            px.append(fix.marker_px)
    if len(d) < 5:
        return None
    return statistics.median(d), statistics.median(a), statistics.median(px), float(np.std(d))


def main():
    cam = FrameGrabber()
    f0, source = load_focal(cam.width)
    finder = BeaconFinder(cam.width, cam.height, focal_px=f0)
    print(f"camera {cam.width}x{cam.height}, current focal length {f0:.0f} px [{source}]")
    print(f"marker size in vision_settings.py is {S.MARKER_SIZE_M / 0.0254:.2f} in, make sure your print matches\n")
    rows = []
    try:
        while True:
            text = input("distance lens to marker in inches (enter to finish): ").strip()
            if not text:
                break
            try:
                true_in = float(text)
                assert true_in > 0
            except (ValueError, AssertionError):
                print("  type a number like 36")
                continue
            got = read_marker(cam, finder)
            if got is None:
                print("  marker 0 not seen, check its in view and try again")
                continue
            d, a, px, spread = got
            meas_in = d / 0.0254
            f_here = f0 * true_in / meas_in
            rows.append((true_in, meas_in))
            print(f"  read {meas_in:6.2f} in (spread {spread / 0.0254:.2f} in), angle {a:+.2f} deg, "
                  f"marker {px:.0f} px, focal from this spot {f_here:.0f} px")
    finally:
        cam.close()

    if not rows:
        print("nothing measured, nothing saved")
        return
    true = np.array([r[0] for r in rows])
    meas = np.array([r[1] for r in rows])
    # we use the median, far spots read a little long because the marker is so small there
    k = float(np.median(true / meas))
    if len(rows) >= 2 and np.ptp(true) >= 6:
        # a big constant offset usually means the tape doesnt start at the front of the lens
        slope, c = np.polyfit(meas, true, 1)
        if abs(c) > 3.0:
            print(f"\nthe points have a constant offset of {c:+.1f} in, check where the tape starts and re-measure")
    f_new = f0 * k
    print(f"\nnew focal length {f_new:.0f} px (was {f0:.0f}), field of view "
          f"{np.degrees(2 * np.arctan(cam.width / 2 / f_new)):.0f} deg")
    # a normal webcam at 640 wide should be somewhere in this range
    if not 400 <= f_new * 640 / cam.width <= 1300:
        print("that number doesnt look right, check the marker size and the tape. not saving")
        return
    if input("save it to camera_focal.json? [y/n] ").strip().lower().startswith("y"):
        (HERE / S.FOCAL_FILE).write_text(json.dumps(
            {"width": cam.width, "height": cam.height, "focal_px": round(f_new, 1),
             "points_in": [[float(t), round(float(m), 2)] for t, m in rows],
             "saved": time.strftime("%Y-%m-%d %H:%M")}, indent=2) + "\n")
        print("saved, beacon_vision.py will use it next time")


if __name__ == "__main__":
    main()
