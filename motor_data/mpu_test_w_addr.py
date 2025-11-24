from ctypes import *

dll = windll.LoadLibrary("./CH347DLLA64.DLL")

dll.CH347OpenDevice.argtypes=[c_ulong]
dll.CH347OpenDevice.restype=c_void_p

dll.CH347I2C_Set.argtypes=[c_ulong,c_ulong]
dll.CH347I2C_Set.restype=c_bool

dll.CH347StreamI2C.argtypes=[c_ulong,c_ulong,c_void_p,c_ulong,c_void_p]
dll.CH347StreamI2C.restype=c_bool

IDX = 0
ADDR = 0x68

handle = dll.CH347OpenDevice(IDX)
dll.CH347I2C_Set(IDX, 0x1)

# Write pointer
print("Pointer write...")
wbuf = (c_ubyte*2)((ADDR<<1)|0, 0x3B)
ok = dll.CH347StreamI2C(IDX, 2, wbuf, 0, None)
print("Pointer:", ok)

# Read address
print("Sending read address...")
raddr = (c_ubyte*1)((ADDR<<1)|1)

rbuf = (c_ubyte*14)()
ok = dll.CH347StreamI2C(IDX, 1, raddr, 14, rbuf)
print("Read:", ok)
print("Data:", list(rbuf))
