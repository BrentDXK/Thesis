#!/usr/bin/env python3
# Samengevoegd: robuuste rotary encoder (pigpio) + PWM trigger + OLED pages + BMI160
# - Encoder: quadrature via pigpio callbacks (glitch filter, steps per notch)
# - Short-press = RUN/PAUSE trigger (GPIO18 PWM); Long-press = volgende pagina (5 schermen)
# - Start FPS = 30; FPS_MIN..MAX = 1..240; Pulse = 1.00 ms
# - BMI160 uitlezen (0x69), complementary roll/pitch, ZUPT, |v|/|x|
# - OLED (SSD1306/SH1106) toont 5 pagina's (FPS / BMI compact / BMI detail / Custom1 / Custom2)

import time
from dataclasses import dataclass
from statistics import mean
from math import atan2, degrees, radians
import numpy as np

import pigpio
from smbus2 import SMBus
from PIL import ImageFont
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, sh1106

# ========= PIN DEFINITIONS =========
TRIG_GPIO = 18      # PWM pin -> SN74AHCT125N -> camera trigger
ENC_A = 17          # Rotary CLK (A)
ENC_B = 27          # Rotary DT  (B)
ENC_BTN = 22        # Rotary push button (active low)

# ========= USER SETTINGS =========
FPS_MIN = 1
FPS_MAX = 240
FPS_STEP = 1                 # stap per detent
PULSE_US = 1000              # 1.00 ms high-time
I2C_ADDRESS = 0x3C           # OLED I2C address
OLED_HZ = 10                 # UI refresh Hz
STEPS_PER_NOTCH = 4          # 4 edges = 1 notch bij de meeste encoders
ENCODER_DIR = -1             # +1 normaal, -1 omgekeerd (zoals jouw “links = lager FPS”)
LONG_MS = 800
RELEASE_COOLDOWN_MS = 250

# ========= PAGES =========
PAGE_FPS, PAGE_BMI1, PAGE_BMI2, PAGE_CUSTOM1, PAGE_CUSTOM2 = 0, 1, 2, 3, 4

# ========= STATE =========
state = {
    "fps": 30,                 # start-FPS
    "running": True,           # trigger actief
    "pulse_us": PULSE_US,
    "page": PAGE_FPS,
    "oled_fps": 0.0,           # UI update rate
}

# ========= pigpio init =========
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio is not running. Start met: sudo systemctl start pigpiod")

# Inputs met pull-ups + glitch filters (debounce)
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

# ========= Trigger (PWM) =========
def _apply_trigger_raw(fps: int, pulse_us: int):
    period_us = 1_000_000.0 / fps
    pulse_us = max(1, min(int(period_us - 1), pulse_us))
    duty_255 = int((pulse_us / period_us) * 255)
    duty_255 = max(0, min(255, duty_255))

    state["fps"] = fps
    state["pulse_us"] = pulse_us

    pi.set_PWM_frequency(TRIG_GPIO, fps)
    pi.set_PWM_dutycycle(TRIG_GPIO, duty_255)

def apply_trigger(fps: int):
    fps = max(FPS_MIN, min(FPS_MAX, int(fps)))
    _apply_trigger_raw(fps, PULSE_US)

def change_fps(delta: int):
    new_fps = max(FPS_MIN, min(FPS_MAX, state["fps"] + delta))
    if new_fps != state["fps"]:
        if state["running"]:
            apply_trigger(new_fps)

# Init trigger
apply_trigger(state["fps"])

# ========= Encoder quadrature decoder (zoals jouw perfect werkende code) =========
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
            change_fps(ENCODER_DIR * (-FPS_STEP))  # links = lager bij ENCODER_DIR=-1
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

# ========= Button: short = RUN/PAUSE, long = next PAGE =========
_btn_pressed_t0 = None
_last_release_t = 0.0

def btn_cb(gpio, level, tick):
    global _btn_pressed_t0, _last_release_t
    now = time.monotonic()

    if level == 0:  # down (active low)
        if (now - _last_release_t) * 1000.0 >= RELEASE_COOLDOWN_MS:
            _btn_pressed_t0 = now
    elif level == 1:  # up
        if _btn_pressed_t0 is None:
            _last_release_t = now
            return
        press_ms = (now - _btn_pressed_t0) * 1000.0
        _btn_pressed_t0 = None
        _last_release_t = now

        if press_ms >= LONG_MS:
            state["page"] = (state["page"] + 1) % 5
        else:
            state["running"] = not state["running"]
            if state["running"]:
                apply_trigger(state["fps"])
            else:
                pi.set_PWM_dutycycle(TRIG_GPIO, 0)

pi.callback(ENC_BTN, pigpio.EITHER_EDGE, btn_cb)

# ========= BMI160 driver =========
REG_CHIP_ID   = 0x00
REG_DATA_GYR  = 0x0C
REG_DATA_ACC  = 0x12
REG_TEMP      = 0x20
REG_ACC_RANGE = 0x41
REG_GYR_RANGE = 0x43
REG_CMD       = 0x7E
CHIP_ID       = 0xD1

@dataclass
class ImuSample:
    ax: float; ay: float; az: float   # g
    gx: float; gy: float; gz: float   # deg/s
    temp_c: float
    t: float

class BMI160:
    def __init__(self, bus_id=1, address=0x69):
        self.bus = SMBus(bus_id)
        self.addr = address
        cid = self.bus.read_byte_data(self.addr, REG_CHIP_ID)
        if cid != CHIP_ID:
            raise RuntimeError(f"BMI160 chip id mismatch: got 0x{cid:02X}, expected 0x{CHIP_ID:02X}")
        # normal power
        self.bus.write_byte_data(self.addr, REG_CMD, 0x11); time.sleep(0.01)  # accel normal
        self.bus.write_byte_data(self.addr, REG_CMD, 0x15); time.sleep(0.01)  # gyro normal
        # ranges
        self.bus.write_byte_data(self.addr, REG_ACC_RANGE, 0x03)  # ±2g
        self.bus.write_byte_data(self.addr, REG_GYR_RANGE, 0x00)  # ±2000 dps
        time.sleep(0.01)
        # scales
        self._acc_lsb_per_g   = 16384.0
        self._gyr_lsb_per_dps = 16.4
        self.t0 = time.monotonic()

    def _read_vec3(self, base):
        d = self.bus.read_i2c_block_data(self.addr, base, 6)
        x = (d[1] << 8) | d[0]
        y = (d[3] << 8) | d[2]
        z = (d[5] << 8) | d[4]
        if x & 0x8000: x -= 65536
        if y & 0x8000: y -= 65536
        if z & 0x8000: z -= 65536
        return x, y, z

    def read(self) -> ImuSample:
        gx, gy, gz = self._read_vec3(REG_DATA_GYR)
        ax, ay, az = self._read_vec3(REG_DATA_ACC)
        t2 = self.bus.read_i2c_block_data(self.addr, REG_TEMP, 2)
        t_raw = (t2[1] << 8) | t2[0]
        if t_raw & 0x8000: t_raw -= 65536
        temp_c = 23.0 + (t_raw / 512.0)
        ax_g = ax / self._acc_lsb_per_g
        ay_g = ay / self._acc_lsb_per_g
        az_g = az / self._acc_lsb_per_g
        gx_dps = gx / self._gyr_lsb_per_dps
        gy_dps = gy / self._gyr_lsb_per_dps
        gz_dps = gz / self._gyr_lsb_per_dps
        return ImuSample(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c, time.monotonic() - self.t0)

# ========= Oriëntatie/Integratie helpers =========
G = 9.80665
def complementary_init():
    complementary_roll_pitch.r = 0.0
    complementary_roll_pitch.p = 0.0

def complementary_roll_pitch(ax, ay, az, gx, gy, dt, alpha=0.98):
    roll_acc  = degrees(atan2(ay, az))
    pitch_acc = degrees(atan2(-ax, (ay*0.0 + az)))
    complementary_roll_pitch.r += gx * dt
    complementary_roll_pitch.p += gy * dt
    r = alpha * complementary_roll_pitch.r + (1 - alpha) * roll_acc
    p = alpha * complementary_roll_pitch.p + (1 - alpha) * pitch_acc
    complementary_roll_pitch.r, complementary_roll_pitch.p = r, p
    return r, p

def remove_gravity(ax, ay, az, roll_deg, pitch_deg):
    r = radians(roll_deg); p = radians(pitch_deg)
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    axw =  cy*ax + sy*sx*ay + sy*cx*az
    ayw =       cx*ay       - sx*az
    azw = -sy*ax + cy*sx*ay + cy*cx*az
    return axw, ayw, azw - 1.0

# ========= OLED pages =========
def draw_page_fps():
    fps = state["fps"]
    pulse_ms = state["pulse_us"] / 1000.0
    run_str = "RUN" if state["running"] else "PAUSE"
    with canvas(device) as draw:
        draw.text((0, 0),  "Camera Trigger", font=font_small, fill=255)
        draw.text((0, 14), f"Status: {run_str}", font=font_small, fill=255)
        draw.text((0, 28), f"FPS: {fps}", font=font_small, fill=255)
        draw.text((0, 42), f"Pulse: {pulse_ms:.2f} ms", font=font_small, fill=255)

def draw_page_bmi1(s, ax, ay, az, gx, gy, gz, roll, pitch, vnorm, xnorm, axw, ayw, azw):
    with canvas(device) as draw:
        draw.text((0, 0),  f"BMI160  T:{s.temp_c:.1f}C @0x69", font=font_small, fill=255)
        draw.text((0, 14), f"Acc g  x:{ax:+.2f} y:{ay:+.2f}", font=font_small, fill=255)
        a_norm = float(np.linalg.norm([axw, ayw, azw]))
        draw.text((0, 26), f"z:{az:+.2f} |aw|:{a_norm:.2f}", font=font_small, fill=255)
        draw.text((0, 38), f"Gyr dps x:{gx:+.0f} y:{gy:+.0f}", font=font_small, fill=255)
        draw.text((0, 50), f"r:{roll:+.1f} p:{pitch:+.1f} |v|:{vnorm:.2f} |x|:{xnorm:.2f}", font=font_small, fill=255)

def draw_page_bmi2(axw, ayw, azw, ax, ay, az, gz, dt, oled_fps):
    with canvas(device) as draw:
        draw.text((0, 0),  "BMI DETAILS", font=font_small, fill=255)
        draw.text((0, 14), f"aw[g] x:{axw:+.2f} y:{ayw:+.2f}", font=font_small, fill=255)
        draw.text((0, 26), f"aw[g] z:{azw:+.2f}", font=font_small, fill=255)
        draw.text((0, 38), f"Gz dps:{gz:+.0f}  |a|:{(np.linalg.norm([ax,ay,az])):.2f}g", font=font_small, fill=255)
        draw.text((0, 50), f"dt:{dt*1000:.1f}ms  OLED:{oled_fps:.1f}fps", font=font_small, fill=255)

def draw_page_custom1():
    with canvas(device) as draw:
        draw.text((0, 0),  "CUSTOM PAGE 1", font=font_small, fill=255)
        draw.text((0, 16), "-> ", font=font_small, fill=255)

def draw_page_custom2():
    with canvas(device) as draw:
        draw.text((0, 0),  "CUSTOM PAGE 2", font=font_small, fill=255)

# ========= Main loop =========
def main():
    imu = BMI160(bus_id=1, address=0x69)
    complementary_init()

    # Bias-calibratie (3s stilhouden)
    axb = []; ayb = []; azb = []; gxb = []; gyb = []; gzb = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        s = imu.read()
        axb.append(s.ax); ayb.append(s.ay); azb.append(s.az)
        gxb.append(s.gx); gyb.append(s.gy); gzb.append(s.gz)
        time.sleep(0.005)
    ax_bias = mean(axb); ay_bias = mean(ayb); az_bias = mean(azb) - 1.0
    gx_bias = mean(gxb); gy_bias = mean(gyb); gz_bias = mean(gzb)

    v = np.zeros(3); x = np.zeros(3)
    last = time.monotonic()
    zupt_thresh = 0.02
    zupt_gain   = 0.2

    next_draw = 0.0
    loop_counter = 0
    loop_t0 = time.monotonic()

    try:
        print("Draai encoder per detent voor FPS; short-press = RUN/PAUSE; long-press = next page. Ctrl+C to stoppen.")
        while True:
            # --- IMU lezen & verwerken ---
            s = imu.read()
            now = time.monotonic()
            dt = max(1e-3, now - last)
            last = now

            ax = s.ax - ax_bias; ay = s.ay - ay_bias; az = s.az - az_bias
            gx = s.gx - gx_bias; gy = s.gy - gy_bias; gz = s.gz - gz_bias

            roll, pitch = complementary_roll_pitch(ax, ay, az, gx, gy, dt)
            axw, ayw, azw = remove_gravity(ax, ay, az, roll, pitch)
            a_ms2 = np.array([axw*G, ayw*G, azw*G])

            v += a_ms2 * dt
            x += v * dt
            if abs(ax)+abs(ay)+abs(az) < zupt_thresh:
                v *= (1.0 - zupt_gain*dt)

            vnorm = float(np.linalg.norm(v))
            xnorm = float(np.linalg.norm(x))

            # --- OLED refresh rate berekenen ---
            loop_counter += 1
            if now - loop_t0 >= 0.5:
                state["oled_fps"] = loop_counter / (now - loop_t0)
                loop_counter = 0
                loop_t0 = now

            # --- OLED tekenen (ge-throttled) ---
            t = time.time()
            if t >= next_draw:
                p = state["page"]
                if p == PAGE_FPS:
                    draw_page_fps()
                elif p == PAGE_BMI1:
                    draw_page_bmi1(s, ax, ay, az, gx, gy, gz, roll, pitch, vnorm, xnorm, axw, ayw, azw)
                elif p == PAGE_BMI2:
                    draw_page_bmi2(axw, ayw, azw, ax, ay, az, gz, dt, state["oled_fps"])
                elif p == PAGE_CUSTOM1:
                    draw_page_custom1()
                else:
                    draw_page_custom2()
                next_draw = t + 1.0 / OLED_HZ

            time.sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop trigger en pigpio netjes opruimen
        pi.set_PWM_dutycycle(TRIG_GPIO, 0)
        pi.set_PWM_frequency(TRIG_GPIO, 0)
        pi.stop()

if __name__ == "__main__":
    main()
