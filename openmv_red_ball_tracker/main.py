import sensor
import image
import time
from pyb import LED
from pyb import UART

# OpenMV H7/H7 R2 UART(3): TX=P4, RX=P5.
# Connect OpenMV P4(TX) -> ESP32-C3 OPENMV_RX_PIN.
# Connect ESP32-C3 OPENMV_TX_PIN -> OpenMV P5(RX).
# Connect GND -> GND.
uart = UART(3, 57600, timeout_char=20)

# Tune this in OpenMV IDE: Tools -> Machine Vision -> Threshold Editor.
# LAB color threshold for a bright red object.
RED_THRESHOLD = (25, 100, 20, 127, 10, 127)
FACE_CASCADE_PATHS = (
    "/rom/haarcascade_frontalface.cascade",
    "frontalface",
    "frontalface.cascade",
)

FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

YAW_MIN = 30
YAW_MAX = 150
PITCH_MIN = 45
PITCH_MAX = 135
YAW_CENTER = 90
PITCH_CENTER = 90

# If the gimbal moves away from the ball, change the related sign to -1.
YAW_SIGN = -1
PITCH_SIGN = 1

# These can be adjusted live from the ESP32 web page.
yaw_kp = 0.012
pitch_kp = 0.012
dead_zone_pixels = 18
max_step_per_update = 1.0
send_interval_ms = 80
control_interval_ms = 80
target_filter_alpha = 0.25
TARGET_FACE = 0
TARGET_RED_CIRCLE = 1
target_mode = TARGET_FACE

MIN_BLOB_PIXELS = 120
MIN_BLOB_AREA = 120
LINK_TIMEOUT_MS = 1200
LED_UPDATE_INTERVAL_MS = 40
RED_BLINK_PERIOD_MS = 1200
RED_ON_MS = 80

yaw = YAW_CENTER
pitch = PITCH_CENTER
last_send_ms = 0
last_control_ms = 0
last_esp32_ms = 0
last_led_update_ms = 0
uart_line = ""
filtered_x = CENTER_X
filtered_y = CENTER_Y
has_filtered_target = False
active_sensor_mode = -1

red_led = LED(1)
green_led = LED(2)


def load_face_cascade():
    for path in FACE_CASCADE_PATHS:
        try:
            cascade = image.HaarCascade(path, stages=25)
            print("Face cascade loaded:", path)
            return cascade
        except Exception:
            pass

    print("Face cascade missing. Reset OpenMV ROMFS or switch web target to red circle.")
    return None


def clamp(value, low, high):
    return max(low, min(high, value))


def largest_blob(blobs):
    best = None
    best_pixels = 0
    for blob in blobs:
        if blob.pixels() > best_pixels:
            best = blob
            best_pixels = blob.pixels()
    return best


def largest_rect(rects):
    best = None
    best_area = 0
    for rect in rects:
        area = rect[2] * rect[3]
        if area > best_area:
            best = rect
            best_area = area
    return best


def configure_sensor_for_target(force=False):
    global active_sensor_mode, has_filtered_target

    if not force and active_sensor_mode == target_mode:
        return

    if target_mode == TARGET_FACE:
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_auto_gain(True)
    else:
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=500)
    active_sensor_mode = target_mode
    has_filtered_target = False


def find_red_circle(img):
    blobs = img.find_blobs(
        [RED_THRESHOLD],
        pixels_threshold=MIN_BLOB_PIXELS,
        area_threshold=MIN_BLOB_AREA,
        merge=True,
        margin=10,
    )

    blob = largest_blob(blobs)
    if not blob:
        return None

    return (blob.rect(), blob.cx(), blob.cy())


def find_face(img):
    if FACE_CASCADE is None:
        return None

    faces = img.find_features(FACE_CASCADE, threshold=0.75, scale_factor=1.25)
    face = largest_rect(faces)
    if not face:
        return None

    x, y, w, h = face
    return (face, x + w // 2, y + h // 2)


def find_target(img):
    if target_mode == TARGET_RED_CIRCLE:
        return find_red_circle(img)
    return find_face(img)


def draw_color(white=False, red=False, green=False):
    if target_mode == TARGET_FACE:
        if white:
            return 255
        if red:
            return 80
        if green:
            return 180
        return 255

    if red:
        return (255, 0, 0)
    if green:
        return (0, 255, 0)
    return (255, 255, 255)


FACE_CASCADE = load_face_cascade()
if FACE_CASCADE is None:
    target_mode = TARGET_RED_CIRCLE


def send_angles(force=False):
    global last_send_ms
    now = time.ticks_ms()
    if not force and time.ticks_diff(now, last_send_ms) < send_interval_ms:
        return
    last_send_ms = now
    uart.write("%d,%d\n" % (int(yaw), int(pitch)))


def apply_config(line):
    global yaw_kp, pitch_kp, dead_zone_pixels, target_mode
    global max_step_per_update, send_interval_ms, control_interval_ms, target_filter_alpha

    parts = line.split(",")
    if (len(parts) != 7 and len(parts) != 8) or parts[0] != "CFG":
        return

    try:
        yaw_kp = clamp(float(parts[1]), 0.002, 2.000)
        pitch_kp = clamp(float(parts[2]), 0.002, 2.000)
        dead_zone_pixels = int(clamp(int(float(parts[3])), 2, 160))
        max_step_per_update = clamp(float(parts[4]), 0.2, 90.0)
        control_interval_ms = int(clamp(int(float(parts[5])), 10, 1000))
        send_interval_ms = control_interval_ms
        target_filter_alpha = clamp(float(parts[6]), 0.05, 1.00)
        if len(parts) == 8:
            target_mode = int(clamp(int(float(parts[7])), TARGET_FACE, TARGET_RED_CIRCLE))
            if FACE_CASCADE is None and target_mode == TARGET_FACE:
                target_mode = TARGET_RED_CIRCLE
    except Exception:
        pass


def read_esp32_uart():
    global uart_line, last_esp32_ms
    while uart.any():
        char = uart.readchar()
        if char < 0:
            return

        c = chr(char)
        if c == "\r":
            continue

        if c == "\n":
            line = uart_line.strip()
            uart_line = ""
            if line == "OK":
                last_esp32_ms = time.ticks_ms()
            elif line.startswith("CFG,"):
                apply_config(line)
            continue

        if len(uart_line) < 64:
            uart_line += c
        else:
            uart_line = ""


def update_link_led():
    global last_led_update_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_led_update_ms) < LED_UPDATE_INTERVAL_MS:
        return
    last_led_update_ms = now

    connected = time.ticks_diff(now, last_esp32_ms) < LINK_TIMEOUT_MS

    if connected:
        red_led.off()
        green_led.off()
    else:
        green_led.off()
        if now % RED_BLINK_PERIOD_MS < RED_ON_MS:
            red_led.on()
        else:
            red_led.off()


sensor.reset()
configure_sensor_for_target(True)
sensor.skip_frames(time=1500)

clock = time.clock()
send_angles(True)

while True:
    clock.tick()
    read_esp32_uart()
    update_link_led()
    configure_sensor_for_target()

    img = sensor.snapshot()
    target = find_target(img)

    if target:
        rect, target_x, target_y = target
        now = time.ticks_ms()
        img.draw_rectangle(rect, color=draw_color(white=True))
        img.draw_cross(target_x, target_y, color=draw_color(white=True))

        if not has_filtered_target:
            filtered_x = target_x
            filtered_y = target_y
            has_filtered_target = True
        else:
            filtered_x = filtered_x * (1.0 - target_filter_alpha) + target_x * target_filter_alpha
            filtered_y = filtered_y * (1.0 - target_filter_alpha) + target_y * target_filter_alpha

        if time.ticks_diff(now, last_control_ms) < control_interval_ms:
            img.draw_cross(int(filtered_x), int(filtered_y), color=draw_color(red=True))
            img.draw_cross(CENTER_X, CENTER_Y, color=draw_color(green=True))
            continue
        last_control_ms = now

        error_x = filtered_x - CENTER_X
        error_y = filtered_y - CENTER_Y

        next_yaw = yaw
        next_pitch = pitch

        if abs(error_x) > dead_zone_pixels:
            next_yaw += YAW_SIGN * error_x * yaw_kp
        if abs(error_y) > dead_zone_pixels:
            next_pitch += PITCH_SIGN * error_y * pitch_kp

        yaw_delta = clamp(next_yaw - yaw, -max_step_per_update, max_step_per_update)
        pitch_delta = clamp(next_pitch - pitch, -max_step_per_update, max_step_per_update)

        yaw = clamp(yaw + yaw_delta, YAW_MIN, YAW_MAX)
        pitch = clamp(pitch + pitch_delta, PITCH_MIN, PITCH_MAX)
        send_angles()
    else:
        has_filtered_target = False

    img.draw_cross(CENTER_X, CENTER_Y, color=draw_color(green=True))
