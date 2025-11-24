from ctypes import *

dll = windll.LoadLibrary("./CH347DLLA64.dll")
dll.CH347OpenDevice.argtypes=[c_uint]
dll.CH347OpenDevice.restype=c_int
dll.CH347I2C_Set.argtypes=[c_uint,c_ulong]
dll.CH347I2C_Set.restype=c_bool
dll.CH347StreamI2C.argtypes=[c_uint,POINTER(c_ubyte),c_ulong,POINTER(c_ubyte),c_ulong]
dll.CH347StreamI2C.restype=c_bool

dev = 0
print("Open:", dll.CH347OpenDevice(dev))
print("Set:", dll.CH347I2C_Set(dev, 100000))

# New Format Test
# Try to write 1 byte to PWR_MGMT_1 (0x6B) at address 0x68
addr = 0x68

buf = (c_ubyte * 4)(
    addr,   # 0 - I2C 7-bit address
    1,      # 1 - write length
    0,      # 2 - read length
    0x6B    # 3 - register
)

ok = dll.CH347StreamI2C(dev, buf, 4, None, 0)
print("StreamI2C (new header format) =", ok)
