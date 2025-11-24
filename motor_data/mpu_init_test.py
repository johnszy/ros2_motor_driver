from ctypes import *
import time
import struct

dll = windll.LoadLibrary("./CH347DLLA64.DLL")

dll.CH347OpenDevice.argtypes=[c_ulong]
dll.CH347OpenDevice.restype=c_void_p

dll.CH347I2C_Set.argtypes=[c_ulong,c_ulong]
dll.CH347I2C_Set.restype=c_bool

dll.CH347StreamI2C.argtypes=[c_ulong,c_ulong,c_void_p,c_ulong,c_void_p]
dll.CH347StreamI2C.restype=c_bool

IDX = 0
ADDR = 0x68

def write_reg(reg, val):
    buf = (c_ubyte*3)((ADDR<<1), reg, val)
    ok = dll.CH347StreamI2C(IDX, 3, buf, 0, None)
    print(f"write 0x{reg:02X}={val}: {ok}")
    return ok

def read_bytes(reg, length):
    # pointer write
    wbuf = (c_ubyte*2)((ADDR<<1), reg)
    ok = dll.CH347StreamI2C(IDX, 2, wbuf, 0, None)
    if not ok:
        print("Pointer write failed")
        return None

    # pure read
    rbuf = (c_ubyte*length)()
    ok = dll.CH347StreamI2C(IDX, 0, None, length, rbuf)
    print("Read ok:", ok)
    return list(rbuf)

handle = dll.CH347OpenDevice(IDX)
dll.CH347I2C_Set(IDX, 0x1)

print(">>> INITIALIZING CLONE MPU (0x70)")

write_reg(0x6B, 0x01)  # PWR_MGMT_1: use PLL
write_reg(0x6C, 0x00)  # PWR_MGMT_2: enable all axes
write_reg(0x1A, 0x03)  # CONFIG
write_reg(0x1B, 0x00)  # GYRO_CONFIG
write_reg(0x1C, 0x00)  # ACCEL_CONFIG
write_reg(0x1D, 0x03)  # ACCEL_CONFIG2

time.sleep(0.1)

print("\n>>> Reading 14 bytes from ACCEL_XOUT_H")
data = read_bytes(0x3B, 14)
print("DATA:", data)
