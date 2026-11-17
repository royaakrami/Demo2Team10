# this sends the vision results from the pi to the arduino over i2c
# the packet layout is in I2C_PROTOCOL.md
# wiring is the same as the mini project: pi sda to a4, scl to a5, gnd to gnd

import struct

from smbus2 import SMBus

ARD_ADDR = 8
ARROW_NONE, ARROW_LEFT, ARROW_RIGHT = 0, 1, 2
STATE_NAMES = ["WAIT", "SEARCH", "ALIGN", "APPROACH", "FINAL_TURN", "DONE", "FAULT"]


def build_packet(seq, seen, angle_deg=0.0, dist_m=0.0, arrow=ARROW_NONE):
    # 7 bytes: seq, flags, angle in 0.01 deg (int16), distance in mm (uint16), xor checksum
    # angle is left positive, distance is camera to marker center
    # when the marker isnt seen everything is 0
    flags = ((1 | ((arrow & 0x03) << 1)) if seen else 0)
    angle = max(-32768, min(32767, round(angle_deg * 100))) if seen else 0
    dist = max(0, min(65535, round(dist_m * 1000))) if seen else 0
    body = struct.pack("<BBhH", seq & 0xFF, flags, angle, dist)
    check = 0
    for b in body:
        check ^= b
    return list(body) + [check]


class ArduinoLink:
    def __init__(self, bus=1, addr=ARD_ADDR):
        self.bus = SMBus(bus)
        self.addr = addr
        self.seq = 0
        self.errors = 0

    def send_fix(self, seen, angle_deg=0.0, dist_m=0.0, arrow=ARROW_NONE):
        # sends one frame, gives back false if the arduino didnt answer
        data = build_packet(self.seq, seen, angle_deg, dist_m, arrow)
        self.seq = (self.seq + 1) & 0xFF
        try:
            # the 0 is an offset byte the arduino throws away (same as the mini project)
            self.bus.write_i2c_block_data(self.addr, 0, data)
            return True
        except OSError:
            self.errors += 1
            return False

    def read_status(self):
        # reads one byte back: (state name, turn decision, is type 2) or None
        try:
            b = self.bus.read_byte(self.addr)
        except OSError:
            return None
        state = STATE_NAMES[b & 0x0F] if (b & 0x0F) < len(STATE_NAMES) else "?"
        turn = {0: "-", 1: "LEFT", 2: "RIGHT"}[(b >> 4) & 0x03] if ((b >> 4) & 0x03) < 3 else "?"
        return state, turn, bool(b & 0x40)

    def close(self):
        self.bus.close()
