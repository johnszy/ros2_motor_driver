from ctypes import *
import struct
import time

dll = windll.LoadLibrary("./CH347DLLA64.DLL")

dll.CH347OpenDevice.argtypes=[c_ulong]
dll.CH347OpenDevice.restype=c_void_p

dll.CH347I2C_Set.argtypes=[c_ulong,c_ulong]
dll.CH347I2C_Set.restype=c_bool

dll.CH347StreamI2C.argtypes=[c_ulong,c_ulong,c_void_p,c_ulong,c_void_p]
dll.CH347StreamI2C.restype=c_bool

IDX = 0
I2C_ADDR = 0x0A   # your PIC I2C address

# ---- I2C helpers ----

def write_bytes(reg, data):
    """
    Write arbitrary number of bytes starting at register.
    """
    wbuf = (c_ubyte * (2 + len(data)))()
    wbuf[0] = (I2C_ADDR << 1) | 0
    wbuf[1] = reg
    for i, b in enumerate(data):
        wbuf[2+i] = b

    ok = dll.CH347StreamI2C(IDX, len(wbuf), wbuf, 0, None)
    if not ok:
        raise IOError(f"Write failed @ reg {reg}")
    return ok


def read_bytes(reg, length):
    """
    PIC16F1619 requires:
    1) write reg pointer with addr|W
    2) read using explicit addr|R
    """
    # Step 1 — pointer write
    ptr = (c_ubyte*2)((I2C_ADDR<<1)|0, reg)
    ok = dll.CH347StreamI2C(IDX, 2, ptr, 0, None)
    if not ok:
        raise IOError("Pointer write failed")

    # Step 2 — explicit read address
    raddr = (c_ubyte*1)((I2C_ADDR<<1)|1)
    rbuf  = (c_ubyte*length)()
    ok = dll.CH347StreamI2C(IDX, 1, raddr, length, rbuf)
    if not ok:
        raise IOError("Read failed")

    return bytes(rbuf)


def read_u16(reg):
    lo, hi = read_bytes(reg, 2)
    return (hi << 8) | lo


def write_u16(reg, value):
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    write_bytes(reg, [lo, hi])


# ---- MAIN TEST ----

print("Opening CH347…")
handle = dll.CH347OpenDevice(IDX)
dll.CH347I2C_Set(IDX, 0x1)  # 100kHz

print("\n---- PIC16F1619 motor_reg[ ] test ----")

# 1) Scan register 0..15
print("\nInitial PIC motor_regs dump:")
vals = read_bytes(0, 16)
print(" ".join(f"{v:02X}" for v in vals))

# 2) Write a known test value into reg 4 (P_GAIN MSB/LSB)
print("\nWriting 0x1234 into reg[4]/reg[5]…")
write_u16(4, 0x1234)

# 3) Read it back
val = read_u16(4)
print(f"Readback = 0x{val:04X}")

# 4) Write another multibyte value into reg 0 (Velocity target)
print("\nWriting 0x055A into reg[0]/reg[1]…")
write_u16(0, 0x055A)

# 5) Readback
val2 = read_u16(0)
print(f"Velocity target = 0x{val2:04X}")

# Final dump
print("\nFinal PIC motor_regs[] dump:")
vals = read_bytes(0, 16)
print(" ".join(f"{v:02X}" for v in vals))
print("\nTest complete.")
