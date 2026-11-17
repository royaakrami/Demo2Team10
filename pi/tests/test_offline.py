#!/usr/bin/env python3
# this tests the vision code without a camera, arduino or robot
# it puts the real beacon pngs into fake camera frames at known distances and angles
# and checks the angle, distance and arrow, then checks the packet bytes
# run from the pi folder: python3 tests/test_offline.py
import math
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

PI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI_DIR))
import vision_settings as S                    # noqa: E402
from arduino_link import build_packet          # noqa: E402
from beacon_detect import BeaconFinder         # noqa: E402

W, H, F = 640, 480, 770.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
rng = np.random.default_rng(1)


def load_beacon(name):
    rgba = cv2.imread(str(PI_DIR.parent / name), cv2.IMREAD_UNCHANGED)
    assert rgba is not None, f"{name} not found next to the pi folder"
    # the PNGs have a transparent background: put them on white paper like a real print, with margin
    alpha = rgba[..., 3:4].astype(np.float32) / 255.0
    img = (rgba[..., :3] * alpha + 255 * (1 - alpha)).astype(np.uint8)
    img = cv2.copyMakeBorder(img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # find the marker's black square in the image so we know the pixel -> meter scale
    # the marker is the big black block; the arrow outline is thin, so take the densest columns
    cols = np.where((gray < 60).sum(axis=0) > 0.6 * (img.shape[0] - 160))[0]
    mx0, mx1 = cols.min(), cols.max() + 1
    return img, mx0, mx1


def render(beacon, mx0, mx1, dist_ft, angle_deg, yaw_deg=0.0, height_m=-0.05, clutter=False):
    # makes a fake camera frame with the beacon at a known angle and distance
    img = beacon
    h_px, w_px = img.shape[:2]
    m_per_px = S.MARKER_SIZE_M / (mx1 - mx0)
    cx_px = (mx0 + mx1) / 2.0                       # marker center in the page image
    cy_px = h_px / 2.0
    # page corners in the page's own frame (meters), origin at the marker center, x right, y down
    pc = np.array([[0, 0], [w_px, 0], [w_px, h_px], [0, h_px]], float)
    local = np.c_[(pc[:, 0] - cx_px) * m_per_px, (pc[:, 1] - cy_px) * m_per_px, np.zeros(4)]
    yaw = math.radians(yaw_deg)
    R = np.array([[math.cos(yaw), 0, math.sin(yaw)], [0, 1, 0], [-math.sin(yaw), 0, math.cos(yaw)]])
    d = dist_ft * 0.3048
    a = math.radians(angle_deg)
    horiz = math.sqrt(max(d * d - height_m * height_m, 1e-6))
    center = np.array([-horiz * math.sin(a), height_m, horiz * math.cos(a)])   # left positive -> -x
    cam_pts = (R @ local.T).T + center
    proj = (K @ cam_pts.T).T
    proj = proj[:, :2] / proj[:, 2:3]
    Hm = cv2.getPerspectiveTransform(pc.astype(np.float32), proj.astype(np.float32))
    bg = np.full((H, W, 3), (150, 140, 135), np.uint8)
    bg = (bg + rng.normal(0, 6, bg.shape)).clip(0, 255).astype(np.uint8)
    if clutter:   # a red object on the LEFT and a green one on the RIGHT, far from the beacon
        cv2.rectangle(bg, (5, 5), (60, 60), (30, 30, 220), -1)
        cv2.rectangle(bg, (580, 420), (635, 475), (40, 180, 40), -1)
    mask = cv2.warpPerspective(np.full(img.shape[:2], 255, np.uint8), Hm, (W, H))
    warped = cv2.warpPerspective(img, Hm, (W, H), flags=cv2.INTER_AREA)
    out = np.where(mask[..., None] > 0, warped, bg)
    out = cv2.GaussianBlur(out, (3, 3), 0.7)
    # dim, warmish room light + jpeg like a webcam
    out = (out.astype(np.float32) * np.array([0.80, 0.85, 0.95]) + rng.normal(0, 3, out.shape)).clip(0, 255)
    ok, jpg = cv2.imencode(".jpg", out.astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 80])
    # is the whole marker in frame? is the whole page (marker + arrow) in frame?
    mk = np.array([[mx0, 80], [mx1, 80], [mx1, h_px - 80], [mx0, h_px - 80]], np.float32).reshape(-1, 1, 2)
    mk = cv2.perspectiveTransform(mk, Hm).reshape(-1, 2)
    inside = lambda q: bool(((q[:, 0] > 2) & (q[:, 0] < W - 2) & (q[:, 1] > 2) & (q[:, 1] < H - 2)).all())
    return cv2.imdecode(jpg, cv2.IMREAD_COLOR), inside(mk), inside(proj)


def main():
    finder = BeaconFinder(W, H, focal_px=F)
    beacons = {1: load_beacon("leftarrow.png"), 2: load_beacon("rightarrow.png")}
    worst_a = worst_d = worst_close = 0.0
    misses = wrong_arrow = no_arrow = total = 0
    print(" ft   true_ang  meas_ang   meas_ft  arrow(want)  px")
    for want, (img, mx0, mx1) in beacons.items():
        for ft in (1.25, 1.75, 2.5, 4.0, 5.0, 6.0):
            for ang in (-20, -10, 0, 10, 20):
                if ft < 1.5 and abs(ang) > 15:
                    continue   # arrow is out of frame that close and that far off
                for yaw in (-25, 0, 25):
                    total += 1
                    frame, marker_in, page_in = render(img, mx0, mx1, ft, ang, yaw, clutter=(yaw == 25))
                    fix = finder.process(frame)
                    if not marker_in:
                        total -= 1
                        continue   # marker partly out of the frame: nothing to find, not a failure
                    if not fix.seen:
                        misses += 1
                        print(f"{ft:4.2f}  {ang:+6.1f}   NOT SEEN (yaw {yaw})")
                        continue
                    ea = abs(fix.angle_deg - ang)
                    ed_pct = abs(fix.dist_m / 0.3048 - ft) / ft * 100
                    worst_a, worst_d = max(worst_a, ea), max(worst_d, ed_pct)
                    if ft <= 2.5:
                        worst_close = max(worst_close, abs(fix.dist_m / 0.3048 - ft) * 12)
                    if fix.arrow == 0 and page_in:
                        no_arrow += 1
                    elif fix.arrow not in (0, want):
                        wrong_arrow += 1
                    if yaw == 0 or (fix.arrow != want and page_in):
                        print(f"{ft:4.2f}  {ang:+6.1f}   {fix.angle_deg:+7.2f}   {fix.dist_m / 0.3048:6.2f}"
                              f"      {fix.arrow}({want})    {fix.marker_px:4.0f}  yaw {yaw:+d}")
    print(f"\n{total} frames: not seen {misses}, arrow wrong {wrong_arrow}, arrow missing {no_arrow}")
    print(f"worst angle error {worst_a:.2f} deg, worst distance error {worst_d:.1f} %, "
          f"worst within 2.5 ft {worst_close:.2f} in (this range sets the stop point)")

    # check the packet bytes the same way Demo2_Beacon.ino decodes them
    for seen, ang, dist, arrow in [(True, 12.34, 1.5, 1), (True, -45.67, 0.381, 2), (False, 3, 2, 1)]:
        b = build_packet(200, seen, ang, dist, arrow)
        assert len(b) == 7
        x = 0
        for v in b[:6]:
            x ^= v
        assert x == b[6], "checksum"
        flags = b[1]
        angle_cdeg = struct.unpack("<h", bytes([b[2], b[3]]))[0]      # (int16)(b2 | b3 << 8)
        dist_mm = b[4] | (b[5] << 8)
        if seen:
            assert flags & 1 and (flags >> 1) & 3 == arrow
            assert abs(angle_cdeg / 100 - ang) < 0.006 and abs(dist_mm / 1000 - dist) < 0.0006
        else:
            assert flags == 0 and angle_cdeg == 0 and dist_mm == 0
    print("packets: layout, sign and checksum match the Arduino decoder")
    ok = misses == 0 and wrong_arrow == 0 and no_arrow == 0 and worst_a < 1.0 and worst_d < 8 and worst_close < 1.0
    print("PASS" if ok else "CHECK THE NUMBERS ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
