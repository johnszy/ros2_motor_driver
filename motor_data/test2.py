from ctypes import *
import time

dll = windll.LoadLibrary("./CH347DLLA64.dll")

dll.CH347OpenDevice.argtypes=[c_uint]
dll.CH347OpenDevice.restype=c_int
dll.CH347I2C_Set.argtypes=[c_uint,c_ulong]
dll.CH347I2C_Set.restype=c_bool
dll.CH347StreamI2C.argtypes=[c_uint,POINTER(c_ubyte),c_ulong,POINTER(c_ubyte),c_ulong]
dll.CH347StreamI2C.restype=c_bool

dev=0

print("Open:", dll.CH347OpenDevice(dev))
print("Set I2C:", dll.CH347I2C_Set(dev,100000))

# ---- TEST A: include address ----
print("Test A (addr included):")
bufA=(c_ubyte*2)(0x68<<1, 0x6B)
okA=dll.CH347StreamI2C(dev, bufA, 2, None, 0)
print("  OK =", okA)

# ---- TEST B: no address ----
print("Test B (no address):")
bufB=(c_ubyte*1)(0x6B)
okB=dll.CH347StreamI2C(dev, bufB, 1, None, 0)
print("  OK =", okB)
