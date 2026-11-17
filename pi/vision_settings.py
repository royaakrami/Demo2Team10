# these are all the settings for the pi vision code
# change the numbers here and run beacon_vision.py again

# camera
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FLIP_HORIZONTAL = False   # true if the preview looks mirrored
FLIP_VERTICAL = False     # true if the camera is upside down (set both flips true for that)
EXPOSURE = 300            # fixed exposure, bright enough to read the arrow (about 4 fps, the TA said thats fine)
                          # 156 gives 15 fps but the arrow gets too dark, None = auto
LOCK_FOCUS = None         # None = leave autofocus on, a number like 30 = fixed focus

# focal length in pixels, quick_focal.py measures it and saves camera_focal.json
# that file is used first, this number is only used if the file is missing
# we measured 666 px on our webcam at 640x480
FOCAL_PX_DEFAULT = 666.0
FOCAL_FILE = "camera_focal.json"

# marker is 6x6 dictionary id 0, the black square is 2 inches
MARKER_ID = 0
MARKER_SIZE_M = 0.0508

# arrow colors in opencv hsv (h is 0-180)
# green arrow on the left of the marker means turn left, red on the right means turn right
GREEN_H = (40, 95)
GREEN_S_MIN = 70
GREEN_V_MIN = 40
RED_H_LOW = 10            # red wraps around so its h <= 10 or h >= 165
RED_H_HIGH = 165
RED_S_MIN = 90
RED_V_MIN = 50
ARROW_MIN_AREA = 0.05     # arrow blob size compared to the marker area (the real arrow is about 0.2)
ARROW_MAX_AREA = 0.80

# arduino link
SEND_TO_ARDUINO = True    # false = camera only, nothing gets sent
I2C_BUS = 1
STATUS_EVERY_S = 0.25     # how often we read the arduino state back

# lcd, skipped if it isnt plugged in
USE_LCD = True
LCD_COLUMNS = 16
LCD_ROWS = 2

# preview window and logging
SHOW_PREVIEW = "auto"     # auto = show it when a monitor is attached
SAVE_LOG = True           # saves logs/vision_<time>.csv with every frame
PRINT_EVERY_S = 0.5
