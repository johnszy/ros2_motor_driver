from ctypes import *
import struct
import time
import math

dll = windll.LoadLibrary("./CH347DLLA64.DLL")

# --- DLL function prototypes ---
dll.CH347OpenDevice.argtypes=[c_ulong]
dll.CH347OpenDevice.restype=c_void_p

dll.CH347I2C_Set.argtypes=[c_ulong,c_ulong]
dll.CH347I2C_Set.restype=c_bool

dll.CH347StreamI2C.argtypes=[c_ulong,c_ulong,c_void_p,c_ulong,c_void_p]
dll.CH347StreamI2C.restype=c_bool

IDX = 0
ADDR = 0x68  # your IMU address

# --- I2C helpers ---
def write_reg(reg, val):
    buf = (c_ubyte*3)((ADDR<<1) | 0, reg, val)
    ok = dll.CH347StreamI2C(IDX, 3, buf, 0, None)
    if not ok:
        raise IOError(f"I2C write failed @ reg 0x{reg:02X}")
    return ok

def read_bytes(reg, length):
    # Step 1: write pointer
    wbuf = (c_ubyte*2)((ADDR<<1)|0, reg)
    ok = dll.CH347StreamI2C(IDX, 2, wbuf, 0, None)
    if not ok:
        raise IOError("Pointer write failed")

    # Step 2: send read address explicitly
    raddr = (c_ubyte*1)((ADDR<<1)|1)
    rbuf = (c_ubyte*length)()
    ok = dll.CH347StreamI2C(IDX, 1, raddr, length, rbuf)
    if not ok:
        raise IOError("Read failed")

    return bytes(rbuf)

def read_word(reg):
    data = read_bytes(reg, 2)
    val = (data[0] << 8) | data[1]
    if val & 0x8000:
        val = -((65535 - val) + 1)
    return val

# --- Initialize device ---
print("Opening CH347...")
handle = dll.CH347OpenDevice(IDX)
dll.CH347I2C_Set(IDX, 0x1)  # 100 kHz

print("Waking IMU...")
write_reg(0x6B, 0x00)  # wake

# Read config FS ranges
accel_cfg = read_bytes(0x1C, 1)[0]
gyro_cfg  = read_bytes(0x1B, 1)[0]

accel_div = {0:16384.0, 1:8192.0, 2:4096.0, 3:2048.0}[(accel_cfg>>3)&3]
gyro_div  = {0:131.0,   1:65.5,   2:32.8,  3:16.4  }[(gyro_cfg>>3)&3]

print(f"Accel FS divisor = {accel_div}")
print(f"Gyro  FS divisor = {gyro_div}")

# --- Gyro bias calibration ---
def calibrate_gyro(samples=200, delay=0.01):
    gx=gy=gz=0
    for _ in range(samples):
        gx += read_word(0x43)
        gy += read_word(0x45)
        gz += read_word(0x47)
        time.sleep(delay)
    return gx/samples/gyro_div, gy/samples/gyro_div, gz/samples/gyro_div

print("Calibrating gyro... keep IMU still")
gx_bias, gy_bias, gz_bias = calibrate_gyro()
print("Gyro bias:", gx_bias, gy_bias, gz_bias)

G_TO_MPS2 = 9.80665

print("\nStreaming IMU data...\n")
while True:
    ax = read_word(0x3B) / accel_div
    ay = read_word(0x3D) / accel_div
    az = read_word(0x3F) / accel_div

    gx = read_word(0x43) / gyro_div - gx_bias
    gy = read_word(0x45) / gyro_div - gy_bias
    gz = read_word(0x47) / gyro_div - gz_bias

    temp = read_word(0x41) / 340.0 + 36.53

    print(f"Accel (m/s²): X={ax*G_TO_MPS2:6.2f} "
          f"Y={ay*G_TO_MPS2:6.2f} Z={az*G_TO_MPS2:6.2f} | "
          f"Gyro (°/s): X={gx:6.2f} Y={gy:6.2f} Z={gz:6.2f} | "
          f"Temp {temp:.2f} °C")
    time.sleep(0.5)
