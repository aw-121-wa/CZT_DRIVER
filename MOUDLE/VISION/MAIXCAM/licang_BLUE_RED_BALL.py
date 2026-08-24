"""MaixCAM2 red/blue ball ROI recognizer.

The script intentionally uses a small ASCII protocol:

* STM32 -> MaixCAM2: ``1`` selects red, ``2`` selects blue.
* MaixCAM2 -> STM32: ``1\\n`` reports one confirmed target.

Thresholds and geometry values near the top of this file are field-calibration
parameters and should be tuned with the B25 illumination LED enabled.
"""

try:
    from maix import camera, display, gpio, image, pinmap, time, touchscreen
    from maix.peripheral import uart
    MAIXPY = True
except Exception:
    camera = display = gpio = image = pinmap = time = touchscreen = uart = None
    MAIXPY = False


# ========================== User Config ==========================

MODE_RED = 1
MODE_BLUE = 2
COLOR_NAMES = {
    MODE_RED: "RED",
    MODE_BLUE: "BLUE",
}

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

# MaixCAM2 official UART4: TX=A21, RX=A22, device=/dev/ttyS4.
UART_DEVICE = "/dev/ttyS4"
UART_BAUDRATE = 115200
UART_READ_CHUNK = 64

# Recognition region. Adjust this value at the competition site if needed.
ROI = [120, 80, 400, 300]

# TODO: Re-calibrate with the B25 illumination LED enabled in the real venue.
RED_THRESHOLD = (0, 80, 40, 80, 10, 80)
# TODO: Re-calibrate with the B25 illumination LED enabled in the real venue.
BLUE_THRESHOLD = (10, 80, -20, 50, -100, -20)
COLOR_THRESHOLDS = {
    MODE_RED: RED_THRESHOLD,
    MODE_BLUE: BLUE_THRESHOLD,
}

PIXELS_THRESHOLD = 700
AREA_THRESHOLD = 900
MIN_AREA = 900
MIN_WIDTH = 20
MIN_HEIGHT = 20
MAX_WIDTH = 260
MAX_HEIGHT = 260
RATIO_MIN = 0.65
RATIO_MAX = 1.35

DETECT_CONFIRM_FRAMES = 3
LOST_RESET_FRAMES = 5

BUTTON_RED = [10, 405, 300, 60]
BUTTON_BLUE = [330, 405, 300, 60]
TOUCH_DEBOUNCE_MS = 250
MAIN_LOOP_SLEEP_MS = 1


# ========================== Runtime State ==========================

current_mode = MODE_RED
detected_latched = False
detected_streak = 0
lost_frames = 0
last_touch_ms = 0
illumination_gpio = None


# ========================== Time and Hardware ==========================

def ticks_ms():
    if MAIXPY and time is not None:
        return time.ticks_ms()
    import time as pytime
    return int(pytime.time() * 1000)


def sleep_ms(milliseconds):
    if MAIXPY and time is not None:
        time.sleep_ms(milliseconds)
    else:
        import time as pytime
        pytime.sleep(milliseconds / 1000.0)


def light_init():
    """Map MaixCAM2 B25 to GPIO output and leave it initially off."""
    global illumination_gpio
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    pinmap.set_pin_function("B25", "B25")
    illumination_gpio = gpio.GPIO("B25", gpio.Mode.OUT)
    light_off()
    return illumination_gpio


def light_on():
    """Turn on the MaixCAM2 onboard illumination LED."""
    if illumination_gpio is None:
        raise RuntimeError("light_init() must be called first")
    illumination_gpio.value(1)


def light_off():
    """Turn off the MaixCAM2 onboard illumination LED."""
    if illumination_gpio is None:
        return
    illumination_gpio.value(0)


def init_uart():
    """Initialize official MaixCAM2 UART4 on A21/A22."""
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    pinmap.set_pin_function("A21", "UART4_TX")
    pinmap.set_pin_function("A22", "UART4_RX")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def init_camera():
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    return camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)


def init_display():
    if not MAIXPY:
        raise RuntimeError("MaixPy modules are not available")
    return display.Display()


def init_touchscreen():
    if not MAIXPY or touchscreen is None:
        return None
    return touchscreen.TouchScreen()


# ========================== Mode and UART Protocol ==========================

def set_mode(mode, source):
    """Set the one shared mode state and re-arm recognition for a new task."""
    global current_mode, detected_latched, detected_streak, lost_frames
    if mode not in (MODE_RED, MODE_BLUE):
        return False
    current_mode = mode
    detected_latched = False
    detected_streak = 0
    lost_frames = 0
    return True


def process_command_bytes(data):
    """Process any ASCII 1/2 bytes without waiting for a complete line."""
    if data is None:
        return
    if isinstance(data, str):
        data = data.encode("ascii", "ignore")
    try:
        values = bytes(data)
    except (TypeError, ValueError):
        return
    for value in values:
        if value == ord("1"):
            set_mode(MODE_RED, "uart")
        elif value == ord("2"):
            set_mode(MODE_BLUE, "uart")


def uart_process(serial):
    """Poll UART once; MaixPy read() without arguments returns immediately."""
    if serial is None:
        return
    try:
        data = serial.read()
    except Exception:
        return
    if data:
        process_command_bytes(data)


# ========================== Blob Detection ==========================

def _blob_value(blob, name, fallback=0):
    value = getattr(blob, name, None)
    if value is None:
        return fallback
    try:
        value = value() if callable(value) else value
        return int(value)
    except (TypeError, ValueError):
        return fallback


def filter_blob(blob):
    """Return blob if it has ball-like geometry, otherwise return None."""
    if blob is None:
        return None
    width = _blob_value(blob, "w")
    height = _blob_value(blob, "h")
    if width < MIN_WIDTH or height < MIN_HEIGHT:
        return None
    if width > MAX_WIDTH or height > MAX_HEIGHT:
        return None
    pixels = _blob_value(blob, "pixels", width * height)
    area = _blob_value(blob, "area", width * height)
    if pixels < PIXELS_THRESHOLD or area < MIN_AREA:
        return None
    ratio = width / float(height)
    if ratio < RATIO_MIN or ratio > RATIO_MAX:
        return None
    return blob


def select_best_blob(blobs):
    candidates = []
    for blob in blobs or []:
        valid = filter_blob(blob)
        if valid is not None:
            area = _blob_value(valid, "area", 0)
            pixels = _blob_value(valid, "pixels", 0)
            candidates.append((area, pixels, valid))
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def detect_ball(img, mode=None):
    """Find the largest ball-like blob only inside ROI."""
    if img is None:
        return None
    selected_mode = current_mode if mode is None else mode
    threshold = COLOR_THRESHOLDS[selected_mode]
    try:
        blobs = img.find_blobs(
            [threshold],
            roi=ROI,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
        )
    except Exception:
        return None
    return select_best_blob(blobs)


def update_detection(blob, serial):
    """Update the consecutive-frame latch and report one confirmed target."""
    global detected_latched, detected_streak, lost_frames
    valid_blob = filter_blob(blob)
    if valid_blob is not None:
        detected_streak += 1
        lost_frames = 0
        if not detected_latched and detected_streak >= DETECT_CONFIRM_FRAMES:
            if serial is not None:
                try:
                    serial.write(b"1\n")
                except Exception:
                    pass
            detected_latched = True
        return

    detected_streak = 0
    if detected_latched:
        lost_frames += 1
        if lost_frames >= LOST_RESET_FRAMES:
            detected_latched = False
            lost_frames = 0


# ========================== Touch and Display ==========================

def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def screen_to_image_point(x, y, image_width, image_height):
    return (
        int(x * image_width / max(1, SCREEN_WIDTH)),
        int(y * image_height / max(1, SCREEN_HEIGHT)),
    )


def touch_process(touch, image_width=CAMERA_WIDTH, image_height=CAMERA_HEIGHT):
    """Poll one touch event and let the last input source select the mode."""
    global last_touch_ms
    if touch is None:
        return
    now = ticks_ms()
    if now - last_touch_ms < TOUCH_DEBOUNCE_MS:
        return
    try:
        point = touch.read()
    except Exception:
        return
    if not point or len(point) < 3 or not point[2]:
        return
    x, y = screen_to_image_point(point[0], point[1], image_width, image_height)
    if point_in_rect(x, y, BUTTON_RED):
        set_mode(MODE_RED, "touch")
        last_touch_ms = now
    elif point_in_rect(x, y, BUTTON_BLUE):
        set_mode(MODE_BLUE, "touch")
        last_touch_ms = now


def _color(name, fallback):
    if image is None:
        return fallback
    return getattr(image, name, fallback)


def _draw_text(img, x, y, text, color):
    try:
        img.draw_string(x, y, text, color)
    except Exception:
        pass


def draw_ui(img, blob):
    """Draw ROI, target geometry, state text, and both touch buttons."""
    if img is None:
        return
    white = _color("COLOR_WHITE", 0xFFFFFF)
    yellow = _color("COLOR_YELLOW", 0xFFFF00)
    red = _color("COLOR_RED", 0xFF0000)
    blue = _color("COLOR_BLUE", 0x0000FF)
    green = _color("COLOR_GREEN", 0x00FF00)
    try:
        img.draw_rect(ROI[0], ROI[1], ROI[2], ROI[3], yellow)
        valid_blob = filter_blob(blob)
        if valid_blob is not None:
            x = _blob_value(valid_blob, "x")
            y = _blob_value(valid_blob, "y")
            width = _blob_value(valid_blob, "w")
            height = _blob_value(valid_blob, "h")
            center_x = x + width // 2
            center_y = y + height // 2
            img.draw_rect(x, y, width, height, green)
            if hasattr(img, "draw_circle"):
                img.draw_circle(center_x, center_y, 5, red)
        mode_name = COLOR_NAMES[current_mode]
        status = "DETECTED" if detected_latched else "SEARCHING"
        _draw_text(img, 10, 10, "MODE: {}".format(mode_name), white)
        _draw_text(img, 10, 35, "STATUS: {}".format(status), white)

        for mode, rect, label, color in (
            (MODE_RED, BUTTON_RED, "RED", red),
            (MODE_BLUE, BUTTON_BLUE, "BLUE", blue),
        ):
            outline = color if current_mode == mode else white
            img.draw_rect(rect[0], rect[1], rect[2], rect[3], outline)
            marker = "[X]" if current_mode == mode else "[ ]"
            _draw_text(img, rect[0] + 20, rect[1] + 12,
                       "{} {}".format(marker, label), outline)
    except Exception:
        pass


# ========================== Main Loop ==========================

def main():
    """Initialize MaixCAM2 and keep vision running until the process exits."""
    if not MAIXPY:
        raise RuntimeError("This script must run on MaixCAM2 with MaixPy")

    # The light is initialized and turned on before the camera starts.
    light_init()
    light_on()
    serial = init_uart()
    cam = init_camera()
    disp = init_display()
    touch = init_touchscreen()

    print("MaixCAM2 red/blue ball ROI recognizer started")
    print("UART4 {} TX=A21 RX=A22".format(UART_DEVICE))
    while True:
        uart_process(serial)
        image_frame = cam.read()
        image_width = image_frame.width() if hasattr(image_frame, "width") else CAMERA_WIDTH
        image_height = image_frame.height() if hasattr(image_frame, "height") else CAMERA_HEIGHT
        touch_process(touch, image_width, image_height)
        blob = detect_ball(image_frame)
        update_detection(blob, serial)
        draw_ui(image_frame, blob)
        disp.show(image_frame)
        sleep_ms(MAIN_LOOP_SLEEP_MS)


class _FakeBlob:
    def __init__(self, x, y, width, height, pixels):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._pixels = pixels

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._width

    def h(self):
        return self._height

    def pixels(self):
        return self._pixels

    def area(self):
        return self._width * self._height


class _FakeSerial:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(bytes(data))


class _FakeImage:
    def __init__(self, blobs):
        self.blobs = blobs
        self.kwargs = None

    def find_blobs(self, thresholds, **kwargs):
        self.kwargs = kwargs
        return self.blobs


def _selftest():
    process_command_bytes(b"1\r\n")
    assert current_mode == 1
    process_command_bytes(b"2\r\n")
    assert current_mode == 2
    process_command_bytes(b"x\r\n")
    assert current_mode == 2

    valid = _FakeBlob(120, 80, 50, 50, 1800)
    smaller = _FakeBlob(150, 90, 30, 30, 900)
    non_square = _FakeBlob(150, 90, 100, 20, 1800)
    assert filter_blob(non_square) is None
    assert select_best_blob([smaller, valid]) is valid
    fake_image = _FakeImage([valid])
    assert detect_ball(fake_image, MODE_RED) is valid
    assert fake_image.kwargs["roi"] == ROI
    assert fake_image.kwargs["merge"] is True

    serial = _FakeSerial()
    set_mode(1, "test")
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n"]
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n"]

    for _ in range(5):
        update_detection(None, serial)
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n", b"1\n"]

    set_mode(1, "new-task")
    for _ in range(3):
        update_detection(valid, serial)
    assert serial.sent == [b"1\n", b"1\n", b"1\n"]
    print("red-blue ball selftest passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
