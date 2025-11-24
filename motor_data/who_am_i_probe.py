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

def reg_read(reg):
    # write pointer
    dll.CH347StreamI2C(IDX, 2, (c_ubyte*2)((ADDR<<1), reg), 0, None)
    # read
    rb = (c_ubyte*1)()
    dll.CH347StreamI2C(IDX, 0, None, 1, rb)
    return rb[0]

regs = [0x75, 0x00, 0x01, 0x02, 0x03]

print("WHO_AM_I probe:")
for r in regs:
    try:
        print(f"Reg 0x{r:02X} = 0x{reg_read(r):02X}")
    except:
        print(f"Reg 0x{r:02X} failed")
