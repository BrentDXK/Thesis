from time import sleep, time
from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106, ssd1306

I2C_ADDRESS = 0x3C  # pas aan indien i2cdetect iets anders toont
I2C_PORT = 1

def make_device():
    serial = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    try:
        return sh1106(serial, rotate=0)
    except Exception:
        return ssd1306(serial, rotate=0)

def load_font():
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 12)
    except Exception:
        return ImageFont.load_default()

def main():
    dev = make_device()
    font = load_font()
    W, H = dev.width, dev.height

    # korte witte flits → bevestigt pixels
    with Image.new("1", (W, H), 1) as img:
        dev.display(img); sleep(0.2)

    # loop met FPS-meter
    t0 = time()
    frames = 0
    while True:
        frames += 1
        dt = time() - t0
        fps = frames / dt if dt > 0 else 0.0

        img = Image.new("1", (W, H), 0)
        d = ImageDraw.Draw(img)
        d.text((2, 2),  "Hello Brent!", fill=255, font=font)
        d.text((2, 18), f"{W}x{H} I2C@0x{I2C_ADDRESS:02X}", fill=255, font=font)
        d.text((2, 34), f"FPS: {fps:.1f}", fill=255, font=font)
        dev.display(img)
        sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # poging tot clear, sommige drivers hebben .clear()
        try:
            dev = make_device()
            img = Image.new("1", (dev.width, dev.height), 0)
            dev.display(img)
        except Exception:
            pass
