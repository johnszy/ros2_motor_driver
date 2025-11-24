from ctypes import *
import time

class USBI2C():
    ch347 = windll.LoadLibrary("./CH347DLLA64.dll")
    def __init__(self, usb_dev = 0, i2c_dev = 0x14):   # default to 0x14 (7-bit)
        self.usb_id   = usb_dev
        self.dev_addr = i2c_dev
        io_mode = 0x0200 | (self.dev_addr & 0x7F)

        if USBI2C.ch347.CH347OpenDevice(self.usb_id) != -1:
            USBI2C.ch347.CH347CloseDevice(self.usb_id)
        else:
            print("USB CH347 Open Failed!")

    # --- write-only helper (uses your existing pattern) ---
    def write_bytes(self, data_bytes):
        """Write-only transaction: data_bytes is an iterable of ints (0..255)."""
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) == -1:
            print("USB CH347 Open Failed!")
            return False

        size = len(data_bytes)
        tcmd = (c_byte * (size + 1))()
        ibuf = (c_byte * 1)()
        tcmd[0] = self.dev_addr           # device address (7-bit)
        for i, b in enumerate(data_bytes):
            tcmd[i+1] = b & 0xFF

        # 6 was used earlier in your write(); keep that.
        USBI2C.ch347.CH347StreamI2C(self.usb_id, 6, tcmd, 0, ibuf)
        USBI2C.ch347.CH347CloseDevice(self.usb_id)
        return True

    # --- read-only helper: read N bytes after a prior write(index) ---
    def read_bytes(self, read_len):
        """Read-only transaction: master requests read_len bytes from slave."""
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) == -1:
            print("USB CH347 Open Failed!")
            return None

        # CH347StreamI2C expects a "rec" buffer for control; earlier you used rec[0]=dev_addr
        rec  = (c_byte * 2)()     # request reads
        ibuf = (c_byte * 2)()     # 2-byte receive buffer

        # rec = (c_byte * 1)()
        # ibuf = (c_byte * read_len)()
        rec[0] = self.dev_addr

        # mode 1 was used earlier for read; keep it here. If this doesn't work,
        # try other mode values (2) or (3) — see debugging notes below.
        USBI2C.ch347.CH347StreamI2C(self.usb_id, 1, rec, read_len, ibuf)
        USBI2C.ch347.CH347CloseDevice(self.usb_id)

        # Return Python bytes-like list for convenience
        return [ibuf[i] & 0xFF for i in range(read_len)]

    # --- high-level read of a 16-bit motor_regs entry ---
    def read_reg16(self, reg_index):
        if not (0 <= reg_index < 8):
            raise ValueError("reg_index out of range")

        if USBI2C.ch347.CH347OpenDevice(self.usb_id) == -1:
            print("USB CH347 Open Failed!")
            return None

        # Buffer for 1 write byte (register index)
        write_buf = (c_byte * 1)()
        write_buf[0] = reg_index

        # Buffer for read bytes
        read_buf = (c_byte * 2)()

        # Mode = 0x0200 OR address
        io_mode = 0x0200 | (self.dev_addr << 1)

        res = USBI2C.ch347.CH347StreamI2C(
            self.usb_id,
            io_mode,
            write_buf,
            2,     # bytes to read
            read_buf
        )

        USBI2C.ch347.CH347CloseDevice(self.usb_id)

        print("Stream result:", res)
        print("Raw read:", [hex(read_buf[i] & 0xFF) for i in range(2)])

        return ((read_buf[0] & 0xFF) << 8) | (read_buf[1] & 0xFF)





if __name__ == "__main__":

    q = USBI2C(i2c_dev=0x14)
    for i in range(8):
        print(i, q.read_reg16(i))

##
    q = USBI2C(i2c_dev=0x14)

    # read motor_regs[4]
    # val = q.read_reg16(4)
    # time.sleep(.1)
    # print("motor_regs[4] =", val)

    # for i in range(8):
    #     print("reg", i, "=", q.read_reg16(i))
    #     time.sleep(.1)
