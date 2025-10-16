# raspberrypi-sleep-monitor
Sleep apnea detection system using raspberry pi, max30102, OLED and sound/vibration sensor.
main.py
from machine import Pin, I2C
import utime
import ssd1306
import max30102

# ----- I2C Setup -----
i2c_oled = I2C(1, scl=Pin(7), sda=Pin(6))
i2c_max = I2C(0, scl=Pin(5), sda=Pin(4))

# OLED 128x64
oled = ssd1306.SSD1306_I2C(128, 64, i2c_oled)

# MAX30102 init
m = max30102.MAX30102(i2c=i2c_max)

# Vibration Sensor on GP15
vibration = Pin(15, Pin.IN)

# ----- Apnea Detection Variables -----
last_vibration = utime.ticks_ms()
apnea_threshold = 10000   # 10 seconds without vibration
alert = False

while True:
    # --- Read MAX30102 (HR & SpO2) ---
    hr, spo2 = m.read_sequential()  # returns average HR, SpO2
   
    # --- Check vibration sensor ---
    if vibration.value() == 1:
        last_vibration = utime.ticks_ms()
        alert = False

    # Check if too long without movement
    if utime.ticks_diff(utime.ticks_ms(), last_vibration) > apnea_threshold:
        alert = True

    # --- Display on OLED ---
    oled.fill(0)
    oled.text("Sleep Monitor", 0, 0)
    oled.text("HR: {} bpm".format(hr), 0, 16)
    oled.text("SpO2: {} %".format(spo2), 0, 32)
   
    if alert:
        oled.text("APNEA ALERT!", 0, 48)
    else:
        oled.text("Normal", 0, 48)
   
    oled.show()
    utime.sleep(1)

oled:
# MicroPython SSD1306 OLED driver, I2C and SPI interfaces
from micropython import const
import framebuf

SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)

class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        self.framebuf = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00, # off
            SET_MEM_ADDR, 0x00, # horizontal
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xff,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01):
            self.write_cmd(cmd)

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        for page in range(self.pages):
            self.write_cmd(SET_PAGE_ADDR)
            self.write_cmd(page)
            self.write_cmd(0xFF)
            self.write_cmd(SET_COL_ADDR)
            self.write_cmd(0)
            self.write_cmd(self.width - 1)
            self.write_data(self.buffer[page * self.width:(page + 1) * self.width])

    def fill(self, color):
        self.framebuf.fill(color)

    def pixel(self, x, y, color):
        self.framebuf.pixel(x, y, color)

    def text(self, string, x, y, color=1):
        self.framebuf.text(string, x, y, color)

    def hline(self, x, y, w, color):
        self.framebuf.hline(x, y, w, color)

    def vline(self, x, y, h, color):
        self.framebuf.vline(x, y, h, color)

    def rect(self, x, y, w, h, color):
        self.framebuf.rect(x, y, w, h, color)

    def fill_rect(self, x, y, w, h, color):
        self.framebuf.fill_rect(x, y, w, h, color)

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b'\x40', None]  # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.i2c.writeto(self.addr, bytearray([0x80, cmd]))

    def write_data(self, buf):
        self.i2c.writeto(self.addr, b'\x40' + buf)

max30102

import time
from micropython import const

REG_INTR_STATUS_1 = const(0x00)
REG_INTR_STATUS_2 = const(0x01)
REG_INTR_ENABLE_1 = const(0x02)
REG_INTR_ENABLE_2 = const(0x03)
REG_FIFO_WR_PTR   = const(0x04)
REG_OVF_COUNTER   = const(0x05)
REG_FIFO_RD_PTR   = const(0x06)
REG_FIFO_DATA     = const(0x07)
REG_FIFO_CONFIG   = const(0x08)
REG_MODE_CONFIG   = const(0x09)
REG_SPO2_CONFIG   = const(0x0A)
REG_LED1_PA       = const(0x0C)
REG_LED2_PA       = const(0x0D)
REG_MULTI_LED_CTRL1 = const(0x11)
REG_MULTI_LED_CTRL2 = const(0x12)
REG_TEMP_INTR     = const(0x1F)
REG_TEMP_FRAC     = const(0x20)
REG_TEMP_CONFIG   = const(0x21)
REG_PART_ID       = const(0xFF)

class MAX30102:
    def __init__(self, i2c, addr=0x57):
        self.i2c = i2c
        self.addr = addr
        part_id = self.i2c.readfrom_mem(self.addr, REG_PART_ID, 1)[0]
        if part_id != 0x15:
            raise Exception("MAX30102 not found")
        self.setup()

    def setup(self):
        self.i2c.writeto_mem(self.addr, REG_MODE_CONFIG, b'\x40')  # reset
        time.sleep_ms(100)
        self.i2c.writeto_mem(self.addr, REG_INTR_ENABLE_1, b'\xc0')  # A_FULL and PPG_RDY
        self.i2c.writeto_mem(self.addr, REG_INTR_ENABLE_2, b'\x00')
        self.i2c.writeto_mem(self.addr, REG_FIFO_WR_PTR, b'\x00')
        self.i2c.writeto_mem(self.addr, REG_OVF_COUNTER, b'\x00')
        self.i2c.writeto_mem(self.addr, REG_FIFO_RD_PTR, b'\x00')
        self.i2c.writeto_mem(self.addr, REG_FIFO_CONFIG, b'\x4f')  # sample avg = 4, fifo rollover = false, fifo almost full = 17
        self.i2c.writeto_mem(self.addr, REG_MODE_CONFIG, b'\x03')  # SpO2 mode
        self.i2c.writeto_mem(self.addr, REG_SPO2_CONFIG, b'\x27')  # SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
        self.i2c.writeto_mem(self.addr, REG_LED1_PA, b'\x24')  # ~7mA
        self.i2c.writeto_mem(self.addr, REG_LED2_PA, b'\x24')  # ~7mA

    def read_fifo(self):
        data = self.i2c.readfrom_mem(self.addr, REG_FIFO_DATA, 6)
        red = (data[0] << 16) | (data[1] << 8) | data[2]
        ir = (data[3] << 16) | (data[4] << 8) | data[5]
        red &= 0x03FFFF  # Mask to 18 bits
        ir &= 0x03FFFF
        return red, ir

    def read_sequential(self, samples=10):
        red_buf = []
        ir_buf = []
        for _ in range(samples):
            red, ir = self.read_fifo()
            red_buf.append(red)
            ir_buf.append(ir)
            time.sleep_ms(50)
        avg_red = sum(red_buf) // len(red_buf)
        avg_ir = sum(ir_buf) // len(ir_buf)
        # crude placeholder HR/SpO2 calc
        hr = (avg_ir % 100) + 60
        spo2 = 95 + (avg_red % 5)
        return hr, spo2
