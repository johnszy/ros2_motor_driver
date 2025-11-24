from ctypes import *
import time

dll = windll.LoadLibrary("./CH347DLLA64.DLL")

dll.CH347OpenDevice.argtypes=[c_ulong]
dll.CH347OpenDevice.restype=c_void_p

dll.CH347I2C_Set.argtypes=[c_ulong,c_ulong]
dll.CH347I2C_Set.restype=c_bool

dll.CH347StreamI2C.argtypes=[c_ulong,c_ulong,c_void_p,c_ulong,c_void_p]
dll.CH347StreamI2C.restype=c_bool

IDX = 0
ADDR = 0x68
REG = 0x3B   # ACCEL_XOUT_H
print("Open:", dll.CH347OpenDevice(IDX))
print("I2C Set:", dll.CH347I2C_Set(IDX,0x1))

# STEP 1 — write pointer only
print("Writing register pointer...")
wbuf = (c_ubyte*2)((ADDR<<1)|0, REG)
ok = dll.CH347StreamI2C(IDX, 2, wbuf, 0, None)
print("Write pointer:", ok)

# STEP 2 — now pure read (NO register included)
print("Reading 14 bytes...")
rbuf = (c_ubyte*14)()
ok = dll.CH347StreamI2C(IDX, 0, None, 14, rbuf)
print("Read:", ok)
print("Data:", list(rbuf))
