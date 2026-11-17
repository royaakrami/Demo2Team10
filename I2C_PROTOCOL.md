# Demo 2 I2C protocol

# Demo 2 - How to work with I2C

So the Arduino is the I2C daughter and it's at address 8. This is wired the same as Mini Project.
So Pi SDA would go to A4, SCLE goes to A5, adn GND goes to GND. 

# Pi goes to Arduino with a packet for every camefra frame
So Pi sends a packet for each frame even when you dont see the marker
Which is how the arduino is able to understand that the pi is running. 

`bus.write_i2c_block_data(8, 0, data)`, where the 0 is an offset byte the Arduino throws away.

`data` is 7 bytes, little endian:

byte 0 is seq, it's uint8, adn its a frame counter that wraps at 255
byte 1 is flags, it's uint8, adn its where bit=0 means that the marker is seen, bits 1-2 are equalt to arrow where 0 is no arrow and 1 is left and 2 is right
byte 2-3 is angle, it's int16, adn its the angle to the marker center in 0.01 degrees where left is positive
byte 4-5 is dist, it's uint16, adn its the camera to the marker center in mm 
byte 6 is check, it's uint8, adn its xor of the 0-5 byes

when the marker isn't seen, flags, angle and distance are all 0. `arduino_link.build_packet()` builds it.

The Pi sends the raw angle from each frame, and the Arduino does smoothing for itself
The arrow is sent on any frame where it cn be read. The Arduino counts the votes over the whole run.

# Arduino to Pi: so theres one status byte
`bus.read_byte(8)` gives:
- bits 0-3: state (0 WAIT, 1 SEARCH, 2 ALIGN, 3 APPROACH, 4 FINAL_TURN, 5 DONE, 6 FAULT)
- bits 4-5: turn decision (0 not decided yet, 1 left, 2 right)
- bit 6: 1 if it's set to trial type 2

The Pi  all of shows this on the LCD.
