#!/usr/bin/env python3
# this runs the real beacon_vision.py loop with a fake camera and a fake i2c bus
# it saves the bytes it sent so arduino_rx_test.cpp can decode them with the arduino code
import sys
import types
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import test_offline as T   # noqa: E402

img, a, b = T.load_beacon("leftarrow.png")
blank = cv2.cvtColor(cv2.cvtColor(T.render(img, a, b, 5.0, 80)[0], cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
seen = T.render(img, a, b, 5.0, 12.0)[0]
FRAMES = [blank] * 10 + [seen] * 20


class FakeCap:
    def __init__(self, *a, **k):
        self.i = 0

    def isOpened(self):
        return True

    def set(self, *a):
        return True

    def get(self, *a):
        return 0

    def read(self):
        import time
        time.sleep(0.01)
        if self.i >= len(FRAMES):
            raise KeyboardInterrupt
        f = FRAMES[self.i]
        self.i += 1
        return True, f.copy()

    def release(self):
        pass


writes = []


class FakeBus:
    def __init__(self, bus):
        pass

    def write_i2c_block_data(self, addr, off, data):
        writes.append((addr, off, list(data)))

    def read_byte(self, addr):
        return 0x01 | (1 << 4)   # SEARCH, turn LEFT

    def close(self):
        pass


cv2.VideoCapture = FakeCap
sys.modules["smbus2"] = types.SimpleNamespace(SMBus=FakeBus)
import arduino_link  # noqa: E402
arduino_link.SMBus = FakeBus
import vision_settings as S  # noqa: E402
S.USE_LCD = False
S.SAVE_LOG = False
import beacon_vision  # noqa: E402

# the grabber thread raises KeyboardInterrupt when frames run out; stop the main loop after that
orig_next = beacon_vision.FrameGrabber.next


def next_or_stop(self, last, timeout=1.0):
    got = orig_next(self, last, timeout=0.5)
    if got is None:
        raise KeyboardInterrupt
    return got


beacon_vision.FrameGrabber.next = next_or_stop
sys.argv = ["beacon_vision.py", "--headless"]
beacon_vision.main()

assert writes, "nothing was sent"
assert all(w[0] == 8 and w[1] == 0 and len(w[2]) == 7 for w in writes)
n_seen = sum(w[2][1] & 1 for w in writes)
print(f"{len(writes)} packets captured, {n_seen} with the marker seen")
(HERE / "captured_packets.txt").write_text("\n".join(" ".join(str(x) for x in [w[1]] + w[2]) for w in writes) + "\n")
