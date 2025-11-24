from ctypes import *
import time

# Load CH347 DLL
ch347 = windll.LoadLibrary("./CH347DLLA64.dll")


class CH347I2C:
    def __init__(self, dev_index=0, speed=1):
        """
        dev_index = CH347 device number (0 = first device)
        speed = I2C clock:
            0 = 20kHz
            1 = 100kHz (default)
            2 = 400kHz
            3 = 750kHz
            4 = 50kHz
            5 = 200kHz
            6 = 1MHz
        """
        self.dev_index = dev_index

        # Open the device
        if ch347.CH347OpenDevice(dev_index) == -1:
            raise IOError("Failed to open CH347 device")

        # Configure I2C speed
        if not ch347.CH347I2C_Set(dev_index, speed):
            raise IOError("Failed to initialize CH347 I2C")

        print("CH347 I2C initialized: dev", dev_index)

    def close(self):
        ch347.CH347CloseDevice(self.dev_index)

    # ---------------------------------------------------------
    # Low-level I2C register read: FIRST write register address
    # ---------------------------------------------------------
    def read_registers(self, i2c_addr, reg_addr, read_len):
        """
        Correct CH347 2-step register read:
        1) Write device address + register index
        2) Read N bytes from device
        """

        # ---------- STEP 1: WRITE REGISTER INDEX ----------
        addr_write = (i2c_addr << 1) | 0

        write_buf = (c_ubyte * 2)()
        write_buf[0] = addr_write
        write_buf[1] = reg_addr

        ok = ch347.CH347StreamI2C(
            self.dev_index,
            2, cast(write_buf, c_void_p),   # WRITE reg index
            0, None                         # NO READ YET
        )

        if not ok:
            raise IOError(f"I2C reg-select failed at reg {reg_addr}")

        time.sleep(0.001)   # PIC MSSP stability (important)

        # ---------- STEP 2: READ N BYTES ----------
        addr_read = (i2c_addr << 1) | 1

        read_cmd = (c_ubyte * 1)()
        read_cmd[0] = addr_read

        read_buf = (c_ubyte * read_len)()

        ok = ch347.CH347StreamI2C(
            self.dev_index,
            1, cast(read_cmd, c_void_p),    # SEND read address
            read_len, cast(read_buf, c_void_p)
        )

        if not ok:
            raise IOError(f"I2C read failed at reg {reg_addr}")

        return list(read_buf)


    # ---------------------------------------------------------
    # High-level helper: read a 16-bit motor_reg
    # ---------------------------------------------------------
    def read_u16(self, i2c_addr, reg_index):
        raw = self.read_registers(i2c_addr, reg_index, 2)
        return (raw[0] << 8) | raw[1]

    # ---------------------------------------------------------
    # Low-level write: write register + 16-bit word
    # ---------------------------------------------------------
    def write_u16(self, i2c_addr, reg_index, value):
        """
        Write a 16-bit register to the PIC
        """
        addr_write = (i2c_addr << 1) | 0

        write_buf = (c_ubyte * 4)()
        write_buf[0] = addr_write
        write_buf[1] = reg_index
        write_buf[2] = (value >> 8) & 0xFF
        write_buf[3] = value & 0xFF

        ok = ch347.CH347StreamI2C(
            self.dev_index,
            4, cast(write_buf, c_void_p),
            0, None
        )

        if not ok:
            raise IOError(f"I2C write failed at reg {reg_index}")

        return True

