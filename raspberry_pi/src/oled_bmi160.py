#!/usr/bin/env python3
# oled_bmi160_allinone.py — 1 file: BMI160 driver + OLED UI
# I²C: OLED @ 0x3C, BMI160 @ 0x69 (jij zag 69 met i2cdetect)

import time
from dataclasses import dataclass
from statistics import mean
from math import atan2, degrees, radians, sin, cos
import numpy as np

# ---- OLED (luma.oled) ----
from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106, ssd1306

# ---- BMI160 driver (in-file) ----
from smbus2 import SMBus

# BMI160 registers
REG_CHIP_ID   = 0x00  # expected 0xD1
REG_DATA_GYR  = 0x0C  # GYR_X_L..GYR_Z_H
REG_DATA_ACC  = 0x12  # ACC_X_L..ACC_Z_H
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
    t: float                          # s since start (monotonic)

class BMI160:
    def __init__(self, bus_id=1, address=0x69):  # JOUW ADRES = 0x69
        self.bus = SMBus(bus_id)
        self.addr = address

        # verify chip ID
        cid = self.bus.read_byte_data(self.addr, REG_CHIP_ID)
        if cid != CHIP_ID:
            raise RuntimeError(f"BMI160 chip id mismatch: got 0x{cid:02X}, expected 0x{CHIP_ID:02X}")

        # normal power modes
        self.bus.write_byte_data(self.addr, REG_CMD, 0x11); time.sleep(0.01)  # accel normal
        self.bus.write_byte_data(self.addr, REG_CMD, 0x15); time.sleep(0.01)  # gyro normal

        # ranges
        self.bus.write_byte_data(self.addr, REG_ACC_RANGE, 0x03)  # ±2g
        self.bus.write_byte_data(self.addr, REG_GYR_RANGE, 0x00)  # ±2000 dps
        time.sleep(0.01)

        # scale factors
        self._acc_lsb_per_g   = 16384.0
        self._gyr_lsb_per_dps = 16.4

        self.t0 = time.monotonic()

    def _read_vec3(self, base_reg):
        data = self.bus.read_i2c_block_data(self.addr, base_reg, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        if x & 0x8000: x -= 65536
        if y & 0x8000: y -= 65536
        if z & 0x8000: z -= 65536
        return x, y, z

    def read(self) -> ImuSample:
        gx, gy, gz = self._read_vec3(REG_DATA_GYR)
        ax, ay, az = self._read_vec3(REG_DATA_ACC)

        # temperature (relative)
        tl = self.bus.read_i2c_block_data(self.addr, REG_TEMP, 2)
        t_raw = (tl[1] << 8) | tl[0]
        if t_raw & 0x8000: t_raw -= 65536
        temp_c = 23.0 + (t_raw / 512.0)

        ax_g = ax / self._acc_lsb_per_g
        ay_g = ay / self._acc_lsb_per_g
        az_g = az / self._acc_lsb_per_g

        gx_dps = gx / self._gyr_lsb_per_dps
        gy_dps = gy / self._gyr_lsb_per_dps
        gz_dps = gz / self._gyr_lsb_per_dps

        return ImuSample(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c, time.monotonic() - self.t0)

    def close(self):
        try: self.bus.close()
        except: pass

# ---- helpers for orientation & gravity removal ----
G = 9.80665

def complementary_init():
    complementary_roll_pitch.r = 0.0
    complementary_roll_pitch.p = 0.0

def complementary_roll_pitch(ax, ay, az, gx, gy, dt, alpha=0.98):
    # accel tilt (deg)
    roll_acc  = degrees(atan2(ay, az))
    pitch_acc = degrees(atan2(-ax, (ay*0.0 + az)))
    # integrate gyro (deg/s → deg)
    complementary_roll_pitch.r += gx * dt
    complementary_roll_pitch.p += gy * dt
    # fuse
    r = alpha * complementary_roll_pitch.r + (1 - alpha) * roll_acc
    p = alpha * complementary_roll_pitch.p + (1 - alpha) * pitch_acc
    complementary_roll_pitch.r, complementary_roll_pitch.p = r, p
    return r, p

def remove_gravity(ax, ay, az, roll_deg, pitch_deg):
    r = radians(roll_deg)
    p = radians(pitch_deg)
    cx, sx = cos(r), sin(r)
    cy, sy = cos(p), sin(p)
    # R_x(r)*R_y(p)
    axw =  cy*ax + sy*sx*ay + sy*cx*az
    ayw =       cx*ay       - sx*az
    azw = -sy*ax + cy*sx*ay + cy*cx*az
    return axw, ayw, azw - 1.0  # verwijder ~1g

# ---- OLED setup ----
I2C_OLED_ADDR = 0x3C
I2C_PORT = 1

def make_oled():
    serial = i2c(port=I2C_PORT, address=I2C_OLED_ADDR)
    try:
        return sh1106(serial, rotate=0)  # wijzig naar rotate=2 als ondersteboven
    except Exception:
        return ssd1306(serial, rotate=0)

def load_font():
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 12)
    except Exception:
        return ImageFont.load_default()

# ---- main app ----
def main():
    dev = make_oled()
    font = load_font()
    W, H = dev.width, dev.height

    imu = BMI160(bus_id=1, address=0x69)  # <- jouw BMI160 adres
    complementary_init()

    # korte witte flits om pixels te checken
    with Image.new("1", (W, H), 1) as img:
        dev.display(img); time.sleep(0.2)

    # bias calibratie (stilhouden ~3s)
    print("Calibrating biases… keep IMU still for 3 seconds")
    axb = []; ayb = []; azb = []; gxb = []; gyb = []; gzb = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        s = imu.read()
        axb.append(s.ax); ayb.append(s.ay); azb.append(s.az)
        gxb.append(s.gx); gyb.append(s.gy); gzb.append(s.gz)
        time.sleep(0.005)
    ax_bias = mean(axb); ay_bias = mean(ayb); az_bias = mean(azb) - 1.0
    gx_bias = mean(gxb); gy_bias = mean(gyb); gz_bias = mean(gzb)

    # integrators
    v = np.zeros(3)  # m/s
    x = np.zeros(3)  # m
    last = time.monotonic()
    zupt_thresh = 0.02  # g-som drempel
    zupt_gain   = 0.2

    while True:
        s = imu.read()
        now = time.monotonic(); dt = max(1e-3, now - last); last = now

        # bias correct
        ax = s.ax - ax_bias
        ay = s.ay - ay_bias
        az = s.az - az_bias
        gx = s.gx - gx_bias
        gy = s.gy - gy_bias
        gz = s.gz - gz_bias

        # roll/pitch
        roll, pitch = complementary_roll_pitch(ax, ay, az, gx, gy, dt)

        # gravity removal & integratie
        axw, ayw, azw = remove_gravity(ax, ay, az, roll, pitch)
        a_ms2 = np.array([axw*G, ayw*G, azw*G])
        v += a_ms2 * dt
        x += v * dt

        # ZUPT (ruw): demp snelheid bij quasi stil
        if abs(ax)+abs(ay)+abs(az) < zupt_thresh:
            v *= (1.0 - zupt_gain*dt)

        # OLED render
        img = Image.new("1", (W, H), 0)
        d = ImageDraw.Draw(img)
        # Regel 1: accel
        d.text((2, 0),  f"ax:{ax:+.2f} ay:{ay:+.2f} az:{az:+.2f} g", 255, font=font)
        # Regel 2: gyro
        d.text((2, 14), f"gx:{gx:+.0f} gy:{gy:+.0f} gz:{gz:+.0f} dps", 255, font=font)
        # Regel 3: ori
        d.text((2, 28), f"r:{roll:+.1f} p:{pitch:+.1f} deg", 255, font=font)
        # Regel 4: snelheid/afstand norm
        d.text((2, 42), f"|v|:{np.linalg.norm(v):.2f} m/s |x|:{np.linalg.norm(x):.2f} m", 255, font=font)

        dev.display(img)
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
