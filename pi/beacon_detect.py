# this finds the beacon in one camera frame
# it gives back if marker 0 was seen, the angle to it (left positive), the distance, and the arrow
# we dont use a checkerboard calibration, the camera matrix is just the focal length with the
# center of the image as the optical center

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import vision_settings as S

ARROW_NONE, ARROW_LEFT, ARROW_RIGHT = 0, 1, 2
ARROW_NAMES = {ARROW_NONE: "-", ARROW_LEFT: "LEFT", ARROW_RIGHT: "RIGHT"}
HERE = Path(__file__).resolve().parent


@dataclass
class Fix:
    seen: bool = False
    angle_deg: float = 0.0
    dist_m: float = 0.0
    arrow: int = ARROW_NONE
    marker_px: float = 0.0        # marker side in pixels, under about 15 px it gets unreliable
    corners: np.ndarray = None    # used for drawing the preview
    arrow_box: tuple = None


def load_focal(width):
    # use the focal length quick_focal.py saved, otherwise the default from the settings
    path = HERE / S.FOCAL_FILE
    try:
        data = json.loads(path.read_text())
        if int(data["width"]) == int(width) and float(data["focal_px"]) > 0:
            return float(data["focal_px"]), "measured (camera_focal.json)"
        return S.FOCAL_PX_DEFAULT * width / 640.0, f"default (camera_focal.json is for width {data['width']})"
    except (OSError, ValueError, KeyError):
        return S.FOCAL_PX_DEFAULT * width / 640.0, "default from vision_settings.py"


def camera_matrix(width, height, focal_px):
    return np.array([[focal_px, 0, width / 2.0], [0, focal_px, height / 2.0], [0, 0, 1]], dtype=np.float64)


def hfov_deg(width, focal_px):
    return math.degrees(2 * math.atan(width / 2.0 / focal_px))


def make_detector():
    aruco = cv2.aruco
    # use the same marker dictionary that the printed markers came from
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    # newer and older opencv versions create the detector a little differently
    if hasattr(aruco, "DetectorParameters"):
        params = aruco.DetectorParameters()
    else:
        params = aruco.DetectorParameters_create()
    # refine the corner locations for a more precise angle measurement
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    # a lower perimeter rate lets it find the marker at 6 ft
    params.minMarkerPerimeterRate = 0.015
    if hasattr(aruco, "ArucoDetector"):
        det = aruco.ArucoDetector(dictionary, params)
        return lambda gray: det.detectMarkers(gray)
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def marker_pose(corners, K, size_m=None):
    # solvePnP on the 4 corners gives the marker position in the camera frame
    s = (size_m or S.MARKER_SIZE_M) / 2.0
    # aruco corner order is top left, top right, bottom right, bottom left
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    x, y, z = tvec.ravel()
    # camera x points right, so a marker on the left has negative x and a positive angle
    angle = math.degrees(math.atan2(-x, z))
    return angle, float(np.linalg.norm(tvec))


def _color_masks(hsv):
    h, s, v = cv2.split(hsv)
    green = (h >= S.GREEN_H[0]) & (h <= S.GREEN_H[1]) & (s >= S.GREEN_S_MIN) & (v >= S.GREEN_V_MIN)
    red = ((h <= S.RED_H_LOW) | (h >= S.RED_H_HIGH)) & (s >= S.RED_S_MIN) & (v >= S.RED_V_MIN)
    return green.astype(np.uint8), red.astype(np.uint8)


def _best_blob(mask, x0, y0, marker_area, y_lo, y_hi):
    # biggest blob that is arrow sized and level with the marker
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        frac = area / marker_area
        cy = cents[i][1] + y0
        if not (S.ARROW_MIN_AREA <= frac <= S.ARROW_MAX_AREA and y_lo <= cy <= y_hi):
            continue
        if best is None or area > best[0]:
            x, y, w, h = stats[i, :4]
            best = (area, (int(x + x0), int(y + y0), int(w), int(h)))
    return best


def find_arrow(frame_bgr, corners):
    # looks for green on the left side of the marker and red on the right side
    # on the printed beacon the arrow is about 0.8 marker widths long, so we search 1.5 widths out
    c = np.asarray(corners).reshape(4, 2)
    x0, y0 = c.min(axis=0)
    x1, y1 = c.max(axis=0)
    w, h = x1 - x0, y1 - y0
    if w < 4 or h < 4:
        return ARROW_NONE, None
    H, W = frame_bgr.shape[:2]
    top, bot = int(max(0, y0 - 0.2 * h)), int(min(H, y1 + 0.2 * h))
    marker_area = w * h
    found = {}
    for side, code in (("left", ARROW_LEFT), ("right", ARROW_RIGHT)):
        if side == "left":
            a, b = int(max(0, x0 - 1.5 * w)), int(max(0, x0 - 0.02 * w))
        else:
            a, b = int(min(W, x1 + 0.02 * w)), int(min(W, x1 + 1.5 * w))
        if b - a < 3 or bot - top < 3:
            continue
        hsv = cv2.cvtColor(frame_bgr[top:bot, a:b], cv2.COLOR_BGR2HSV)
        green, red = _color_masks(hsv)
        want = green if side == "left" else red
        blob = _best_blob(want, a, top, marker_area, y0, y1)
        if blob:
            found[code] = blob
    if not found:
        return ARROW_NONE, None
    if len(found) == 2:
        # both sides found something, only trust it if one is clearly bigger
        (ca, ba), (cb, bb) = sorted(found.items(), key=lambda kv: -kv[1][0])
        if ba[0] < 2 * bb[0]:
            return ARROW_NONE, None
        return ca, ba[1]
    code, blob = next(iter(found.items()))
    return code, blob[1]


class BeaconFinder:
    def __init__(self, width, height, focal_px=None):
        self.detect = make_detector()
        if focal_px is None:
            focal_px, self.focal_source = load_focal(width)
        else:
            self.focal_source = "given"
        self.focal_px = focal_px
        self.K = camera_matrix(width, height, focal_px)

    def process(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        if ids is None:
            return Fix()
        for i, mid in enumerate(ids.flatten()):
            if int(mid) != S.MARKER_ID:
                continue
            c = corners[i].reshape(4, 2)
            pose = marker_pose(c, self.K)
            if pose is None:
                return Fix()
            side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
            arrow, box = find_arrow(frame_bgr, c)
            return Fix(True, pose[0], pose[1], arrow, side, c, box)
        return Fix()
