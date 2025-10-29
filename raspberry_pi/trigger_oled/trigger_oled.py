#!/usr/bin/env python3
# Rotary encoder -> FPS control
# Hardware trigger on GPIO18 via pigpio (low-jitter)
# OLED (I2C 0x3C) shows RUN/PAUSE, FPS, pulse width

import time
import pigpio

from PIL import ImageFont
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, sh1106

# ========= PIN DEFINITIONS =========
TRIG_GPIO = 18      # Hardware PWM pin (to SN74AHCT125N -> cameras)
ENC_A = 17          # Rotary CLK
ENC_B = 27          # Rotary DT
ENC_BTN = 22        # Rotary push button (active low)

# ========= USER SETTINGS =========
FPS_MIN = 1
FPS_MAX = 240
FPS_STEP = 1                 # step per detent
PULSE_US = 1000              # 1.0 ms high-time
I2C_ADDRESS = 0x3C           # OLED address
OLED_HZ = 10                 # refresh rate (Hz) for the UI
STEPS_PER_NOTCH = 4          # most encoders give 4 edges per detent
ENCODER_DIR = -1             # +1 normal, -1 invert (so "links = lager FPS")

# ========= STATE =========
state = {
    "fps": 30,
    "running": True,
    "pulse_us": PULSE_US,
}

# ========= pigpio init =========
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio is not running. Do: sudo systemctl start pigpiod")

# Inputs with pull-ups + glitch filters (debounce)
pi.set_mode(ENC_A, pigpio.INPUT)
pi.set_mode(ENC_B, pigpio.INPUT)
pi.set_mode(ENC_BTN, pigpio.INPUT)
pi.set_pull_up_down(ENC_A, pigpio.PUD_UP)
pi.set_pull_up_down(ENC_B, pigpio.PUD_UP)
pi.set_pull_up_down(ENC_BTN, pigpio.PUD_UP)
pi.set_glitch_filter(ENC_A, 2000)   # us
pi.set_glitch_filter(ENC_B, 2000)
pi.set_glitch_filter(ENC_BTN, 6000)

# ========= OLED init =========
def init_oled():
    serial = i2c(port=1, address=I2C_ADDRESS)
    try:
        device = ssd1306(serial, rotate=0)
    except Exception:
        device = sh1106(serial, rotate=0)
    return device

device = init_oled()
try:
    font_small = ImageFont.load_default()
except Exception:
    font_small = None

def draw_oled():
    fps = state["fps"]
    pulse_ms = state["pulse_us"] / 1000.0
    run_str = "RUN" if state["running"] else "PAUSE"
    with canvas(device) as draw:
        draw.text((0, 0),  "Camera Trigger", font=font_small, fill=255)
        draw.text((0, 14), f"Status: {run_str}", font=font_small, fill=255)
        draw.text((0, 28), f"FPS: {fps}", font=font_small, fill=255)
        draw.text((0, 42), f"Pulse: {pulse_ms:.2f} ms", font=font_small, fill=255)

# ========= Trigger control =========
def _apply_trigger_raw(fps: int, pulse_us: int):
    period_us = 1_000_000.0 / fps
    pulse_us = max(1, min(int(period_us - 1), pulse_us))
    # pigpio set_PWM_dutycycle expects 0..255 (not ppm):
    duty_255 = int((pulse_us / period_us) * 255)
    duty_255 = max(0, min(255, duty_255))

    state["fps"] = fps
    state["pulse_us"] = pulse_us

    # Set frequency and duty
    pi.set_PWM_frequency(TRIG_GPIO, fps)
    pi.set_PWM_dutycycle(TRIG_GPIO, duty_255)

    # Debug print (optional)
    # print(f"[TRIGGER] FPS={fps} period={period_us:.1f}us pulse={pulse_us}us duty={duty_255}/255")

def apply_trigger(fps: int):
    fps = max(FPS_MIN, min(FPS_MAX, int(fps)))
    _apply_trigger_raw(fps, PULSE_US)

def change_fps(delta: int):
    new_fps = max(FPS_MIN, min(FPS_MAX, state["fps"] + delta))
    if new_fps != state["fps"]:
        if state["running"]:
            apply_trigger(new_fps)

# Initialize trigger
apply_trigger(state["fps"])

# ========= Robust quadrature decoder =========
# lookup for transitions (prev->curr) yielding -1,0,+1
_lookup = [0, -1, +1, 0,
           +1, 0,  0, -1,
           -1, 0,  0, +1,
            0, +1, -1, 0]

_prev = (pi.read(ENC_A) << 1) | pi.read(ENC_B)
_accum = 0

def _quad_update():
    global _prev, _accum
    curr = (pi.read(ENC_A) << 1) | pi.read(ENC_B)
    key = (_prev << 2) | curr
    _prev = curr
    delta = _lookup[key]
    if delta != 0:
        _accum += delta
        if _accum >= STEPS_PER_NOTCH:
            change_fps(ENCODER_DIR * (-FPS_STEP))
            _accum = 0
        elif _accum <= -STEPS_PER_NOTCH:
            change_fps(ENCODER_DIR * (+FPS_STEP))
            _accum = 0

def enc_a_cb(gpio, level, tick):
    if level != pigpio.TIMEOUT:
        _quad_update()

def enc_b_cb(gpio, level, tick):
    if level != pigpio.TIMEOUT:
        _quad_update()

pi.callback(ENC_A, pigpio.EITHER_EDGE, enc_a_cb)
pi.callback(ENC_B, pigpio.EITHER_EDGE, enc_b_cb)

# ========= Button: toggle RUN/PAUSE =========
def btn_cb(gpio, level, tick):
    if level == 0:  # active-low press
        state["running"] = not state["running"]
        if state["running"]:
            apply_trigger(state["fps"])
        else:
            pi.set_PWM_dutycycle(TRIG_GPIO, 0)

pi.callback(ENC_BTN, pigpio.FALLING_EDGE, btn_cb)

# ========= Main loop =========
try:
    print("Encoder: draai per detent voor FPS; druk voor RUN/PAUSE. Ctrl+C om te stoppen.")
    next_draw = 0.0
    while True:
        now = time.time()
        if now >= next_draw:
            draw_oled()
            next_draw = now + 1.0 / OLED_HZ
        time.sleep(0.005)
except KeyboardInterrupt:
    pass
finally:
    pi.set_PWM_dutycycle(TRIG_GPIO, 0)
    pi.set_PWM_frequency(TRIG_GPIO, 0)
    pi.stop()
