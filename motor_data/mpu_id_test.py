from ctypes import *

# Load DLL
dll = windll.LoadLibrary("./CH347DLLA64.DLL")

# --- prototypes from CH347 manual ---

# HANDLE WINAPI CH347OpenDevice(ULONG DevI);
dll.CH347OpenDevice.argtypes = [c_ulong]
dll.CH347OpenDevice.restype  = c_void_p   # handle, we won't really use it

# BOOL WINAPI CH347I2C_Set(ULONG iIndex, ULONG iMode);
dll.CH347I2C_Set.argtypes    = [c_ulong, c_ulong]
dll.CH347I2C_Set.restype     = c_bool

# BOOL WINAPI CH347StreamI2C(ULONG iIndex, ULONG iWriteLength, PVOID iWriteBuffer,
#                            ULONG iReadLength, PVOID oReadBuffer);
dll.CH347StreamI2C.argtypes  = [c_ulong, c_ulong, c_void_p, c_ulong, c_void_p]
dll.CH347StreamI2C.restype   = c_bool

IDX = 0            # device index
MPU_ADDR = 0x68    # try 0x69 later if needed
WHO_AM_I = 0x75    # should read back 0x68


def main():
    # Open device
    handle = dll.CH347OpenDevice(IDX)
    print("CH347OpenDevice handle:", handle)
    if not handle:
        print("OpenDevice failed")
        return

    # Set I2C speed: mode 0x1 = 100 kHz (per manual)
    ok = dll.CH347I2C_Set(IDX, 0x1)
    print("CH347I2C_Set(100kHz):", ok)
    if not ok:
        print("I2C_Set failed")
        return

    # Prepare write buffer: [I2C_ADDR_WITH_W, REGISTER]
    addr_w = (MPU_ADDR << 1) | 0  # R/W bit = 0 for write
    wbuf = (c_ubyte * 2)(addr_w, WHO_AM_I)
    rbuf = (c_ubyte * 1)()

    # One combined write+read:
    #   write 2 bytes: [addr_w, WHO_AM_I]
    #   read  1 byte  (driver does repeated start + addr with R bit internally)
    ok = dll.CH347StreamI2C(IDX, 2, wbuf, 1, rbuf)
    print("CH347StreamI2C result:", ok)
    if not ok:
        print("StreamI2C returned failure (no ACK or bus error).")
        return

    print("WHO_AM_I read back: 0x%02X" % rbuf[0])


if __name__ == "__main__":
    main()
