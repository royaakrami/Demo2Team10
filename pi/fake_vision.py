#!/usr/bin/env python3
# this tests the arduino over the real i2c link without the camera
# it sends "not seen" at 15 hz like the real vision code does, and you type in fake marker fixes
#
#   v 20 5 R     marker 20 deg to the left, 5 ft away, arrow pointing right
#   v -10 4.5 L  marker 10 deg to the right, 4.5 ft away, arrow pointing left
#   s            read the arduino state
#   q            quit

import threading
import time

from arduino_link import ArduinoLink, ARROW_NONE, ARROW_LEFT, ARROW_RIGHT

FT = 0.3048
link = ArduinoLink()
lock = threading.Lock()
pending = []
running = True


def sender():
    # sends the typed fixes first, otherwise "not seen"
    while running:
        with lock:
            fix = pending.pop(0) if pending else None
        if fix:
            link.send_fix(True, *fix)
        else:
            link.send_fix(False)
        time.sleep(1 / 15)


threading.Thread(target=sender, daemon=True).start()
print("sending 'not seen' at 15 Hz. commands: v <angle_deg> <dist_ft> [L|R], s, q")
while True:
    text = input("> ").strip().split()
    if not text:
        continue
    if text[0] == "q":
        running = False
        break
    if text[0] == "s":
        print(link.read_status(), "I2C errors:", link.errors)
        continue
    if text[0] == "v" and len(text) >= 3:
        arrow = ARROW_NONE
        if len(text) > 3:
            arrow = ARROW_LEFT if text[3].upper() == "L" else ARROW_RIGHT
        fix = (float(text[1]), float(text[2]) * FT, arrow)
        # 3 packets in a row so the arduino locks on
        with lock:
            pending.extend([fix] * 3)
        print("sent", fix)
        continue
    print("use: v <angle_deg> <dist_ft> [L|R], s, q")
link.close()
