# openloop_mtr_test.py
# I2C driver for FT232H board USB -> I2C talking to n20_motor_driver
# Author:  John Szymanski
# Date:    01/16/2026
#
# Refactor: split low-level I2C into FT232H class, and motor register access into MotorCtrl.

import time
import board
import busio


# ----------------------------- Constants / Registers -----------------------------

I2C_ADDRESS_DEFAULT = 0x18

# Register map
REG_MTR_STAT0 = 0x00       # byte 0
REG_MTR_STAT1 = 0x01       # byte 1

REG_MSB_MEAS_RPM = 0x02    # byte 2
REG_LSB_MEAS_RPM = 0x03    # byte 3

REG_MSB_TARGET_RPM = 0x04  # byte 4
REG_LSB_TARGET_RPM = 0x05  # byte 5

REG_MSB_PWM = 0x06         # byte 6
REG_LSB_PWM = 0x07         # byte 7

REG_MSB_KP = 0x08          # byte 8
REG_LSB_KP = 0x09          # byte 9

REG_MSB_KI = 0x0A          # byte 10
REG_LSB_KI = 0x0B          # byte 11

REG_MSB_KD = 0x0C          # byte 12
REG_LSB_KD = 0x0D          # byte 13

REG_TICKS_0 = 0x0E         # byte 14 (LSB)
REG_TICKS_1 = 0x0F         # byte 15
REG_TICKS_2 = 0x10         # byte 16
REG_TICKS_3 = 0x11         # byte 17 (MSB)

REG_MSB_TPR = 0x12         # byte 18
REG_LSB_TPR = 0x13         # byte 19 (MSB)

TICKS_PER_REV = 205  # encoder ticks per wheel revolution
PI = 3.14159


# ---------------------------------- Utilities -----------------------------------

def to_int16(u16: int) -> int:
    u16 &= 0xFFFF
    return u16 - 0x10000 if (u16 & 0x8000) else u16


def to_int32(u32: int) -> int:
    u32 &= 0xFFFFFFFF
    return u32 - 0x100000000 if (u32 & 0x80000000) else u32


# ----------------------------------- FT232H -------------------------------------

class FT232H:
    """
    Thin wrapper around busio.I2C that handles try_lock/unlock and basic
    read/write helpers.
    """

    def __init__(self, scl=board.SCL, sda=board.SDA, *, frequency=100_000):
        self.i2c = busio.I2C(scl, sda, frequency=frequency)

    def _lock(self):
        while not self.i2c.try_lock():
            pass

    def _unlock(self):
        self.i2c.unlock()

    def scan(self):
        self._lock()
        try:
            return list(self.i2c.scan())
        finally:
            self._unlock()

    def write_u8(self, addr: int, reg: int, value: int):
        buf = bytes([(reg & 0xFF), (value & 0xFF)])
        self._lock()
        try:
            self.i2c.writeto(addr, buf)
        finally:
            self._unlock()

    def read_u8(self, addr: int, reg: int) -> int:
        rx = bytearray(1)
        self._lock()
        try:
            # write register pointer, then read one byte
            self.i2c.writeto_then_readfrom(addr, bytes([reg & 0xFF]), rx)
        finally:
            self._unlock()
        return rx[0]

    def read_bytes(self, addr: int, start_reg: int, length: int) -> bytes:
        rx = bytearray(length)
        self._lock()
        try:
            self.i2c.writeto_then_readfrom(addr, bytes([start_reg & 0xFF]), rx)
        finally:
            self._unlock()
        return bytes(rx)

    def write_u16_be(self, addr: int, msb_reg: int, value: int):
        """Write u16 as MSB then LSB to consecutive regs (msb_reg, msb_reg+1)."""
        value &= 0xFFFF
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF
        self.write_u8(addr, msb_reg, msb)
        self.write_u8(addr, msb_reg + 1, lsb)

    def read_u16_be(self, addr: int, msb_reg: int) -> int:
        msb = self.read_u8(addr, msb_reg)
        lsb = self.read_u8(addr, msb_reg + 1)
        return ((msb << 8) | lsb) & 0xFFFF

    def write_u32_le(self, addr: int, lsb_reg: int, value: int, *, inter_byte_delay_s: float = 0.0):
        """
        Write u32 little-endian into 4 consecutive regs (lsb_reg..lsb_reg+3).
        """
        value &= 0xFFFFFFFF
        b = value.to_bytes(4, byteorder="little", signed=False)
        for i, byte in enumerate(b):
            self.write_u8(addr, lsb_reg + i, byte)
            if inter_byte_delay_s > 0:
                time.sleep(inter_byte_delay_s)

    def read_u32_le(self, addr: int, lsb_reg: int) -> int:
        b = self.read_bytes(addr, lsb_reg, 4)  # returns little-endian b0..b3
        return int.from_bytes(b, byteorder="little", signed=False) & 0xFFFFFFFF


# --------------------------------- Motor Ctrl -----------------------------------

class MotorCtrl:
    """
    Higher-level motor register accessors and helpers.
    """

    def __init__(self, ft: FT232H, addr: int = I2C_ADDRESS_DEFAULT):
        self.ft = ft
        self.addr = addr

    # ---- Raw STAT bytes ----
    def write_stat0(self, stat0: int):
        self.ft.write_u8(self.addr, REG_MTR_STAT0, stat0)

    def read_stat0(self) -> int:
        return self.ft.read_u8(self.addr, REG_MTR_STAT0)

    def write_stat1(self, stat1: int):
        self.ft.write_u8(self.addr, REG_MTR_STAT1, stat1)

    def read_stat1(self) -> int:
        return self.ft.read_u8(self.addr, REG_MTR_STAT1)

    # ---- Convenience motion ----
    def go_forward(self):
        # bit0=run, bit1=dir (0=fwd,1=rev)  -> forward+run = 0b00000001
        self.write_stat0(0b00000001)
        print("Motor FORWARD + START bit written")

    def go_reverse(self):
        # reverse+run = 0b00000011
        self.write_stat0(0b00000011)
        print("Motor REVERSE + START bit written")

    def stop(self):
        self.write_stat0(0b00000000)
        print("Motor STOP bit written")

    # ---- Target RPM (u16 BE in regs 0x04/0x05) ----
    def write_target_rpm(self, target_rpm: int):
        self.ft.write_u16_be(self.addr, REG_MSB_TARGET_RPM, target_rpm)

    def read_target_rpm(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_TARGET_RPM)

    # ---- PWM (u16 BE in regs 0x06/0x07) ----
    def write_pwm(self, pwm: int):
        self.ft.write_u16_be(self.addr, REG_MSB_PWM, pwm)

    def read_pwm(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_PWM)

    # ---- PID gains (u16 BE) ----
    def write_kp(self, kp: int):
        self.ft.write_u16_be(self.addr, REG_MSB_KP, kp)

    def read_kp(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_KP)

    def write_ki(self, ki: int):
        self.ft.write_u16_be(self.addr, REG_MSB_KI, ki)

    def read_ki(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_KI)

    def write_kd(self, kd: int):
        self.ft.write_u16_be(self.addr, REG_MSB_KD, kd)

    def read_kd(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_KD)

    # ---- Encoder ticks <-> radians (ticks stored little-endian u32) ----
    def read_ticks_s32(self) -> int:
        ticks_u32 = self.ft.read_u32_le(self.addr, REG_TICKS_0)
        return to_int32(ticks_u32)

    def write_ticks_from_s32(self, ticks_s32: int, *, inter_byte_delay_s: float = 0.1):
        ticks_u32 = ticks_s32 & 0xFFFFFFFF
        self.ft.write_u32_le(self.addr, REG_TICKS_0, ticks_u32, inter_byte_delay_s=inter_byte_delay_s)

    def read_motor_pos_rad(self) -> float:
        ticks_s32 = self.read_ticks_s32()
        #return (2 * PI / TICKS_PER_REV) * ticks_s32
    
        b0 =  self.ft.read_u8(self.addr, REG_TICKS_0)
        b1 =  self.ft.read_u8(self.addr, REG_TICKS_1)
        b2 =  self.ft.read_u8(self.addr, REG_TICKS_2)
        b3 =  self.ft.read_u8(self.addr, REG_TICKS_3)

        ticks_u32 = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
        ticks_s32 = to_int32(ticks_u32)
        rad = (2*PI / TICKS_PER_REV) * ticks_s32
        #print(f"ticks32 = 0x{ticks_u32:08X}  unsigned={ticks_u32}  signed={ticks_s32}\n")
        #print(f'Radians travelled= {rad}\n')
        return rad

    def set_motor_pos_rad(self, rad: float, *, inter_byte_delay_s: float = 0.1):
        ticks_f = (rad * TICKS_PER_REV) / (2 * PI)
        ticks_i = int(round(ticks_f))  # python int (signed, unbounded)
        print(f"ticks_i (python int) = {ticks_i}")

        ticks_u32 = ticks_i & 0xFFFFFFFF  # two's complement for transmit
        print(f"ticks_u32 = {ticks_u32} (0x{ticks_u32:08X})")

        b = ticks_u32.to_bytes(4, byteorder="little", signed=False)
        b0, b1, b2, b3 = b
        print(b0, b1, b2, b3)

        self.ft.write_u32_le(self.addr, REG_TICKS_0, ticks_u32, inter_byte_delay_s=inter_byte_delay_s)

    # ---- Measured RPM (signed s16 in regs 0x02/0x03, BE) ----
    def read_meas_rpm(self) -> int:
        rpm_u16 = self.ft.read_u16_be(self.addr, REG_MSB_MEAS_RPM)
        return to_int16(rpm_u16)

    def write_tpr(self, tpr: int):
        self.ft.write_u16_be(self.addr, REG_MSB_TPR, tpr)

    def read_tpr(self) -> int:
        return self.ft.read_u16_be(self.addr, REG_MSB_TPR)

# ------------------------------------ Main --------------------------------------

def main():
    ft = FT232H()
    mtr = MotorCtrl(ft, I2C_ADDRESS_DEFAULT)

    addrs = ft.scan()
    print("I2C addresses found:", [hex(a) for a in addrs])

    stat0 = mtr.read_stat0()
    print(f"MTR_STAT0 = 0x{stat0:02X}")

    mtr.write_pwm(350)

    #mtr.go_reverse()

    mtr.go_forward()

    pwm = mtr.read_pwm()
    print(f"PWM = {pwm} (0x{pwm:04X})")
    print(f"PWM_MSB = 0x{(pwm >> 8) & 0xFF:02X}")
    print(f"PWM_LSB = 0x{pwm & 0xFF:02X}")

    # --------- motor position and rpms ---------------
    for _ in range(10):
        rad = mtr.read_motor_pos_rad()
        print(f"Radians travelled = {rad:.2f}\n")
        rpm = mtr.read_meas_rpm()
        print(f"Current rpms = {rpm}\n")
        time.sleep(0.3)

    mtr.stop()
    time.sleep(2)

    print(f"KP = {mtr.read_kp()}")
    mtr.write_kp(5210)
    time.sleep(0.2)
    print(f"KP = {mtr.read_kp()}")
    mtr.write_kp(5500)  # back to default

    print(f"KI = {mtr.read_ki()}")
    mtr.write_ki(1212)
    time.sleep(0.2)
    print(f"KI = {mtr.read_ki()}")
    mtr.write_ki(1200)  # back to default

    print(f"KD = {mtr.read_kd()}")
    mtr.write_kd(35)
    time.sleep(0.2)
    print(f"KD = {mtr.read_kd()}")
    mtr.write_kd(50)  # back to default

    print(f"TPR = {mtr.read_tpr()}")
    mtr.write_tpr(21)
    time.sleep(0.2)
    print(f"TPR = {mtr.read_tpr()}")
    mtr.write_tpr(205)  # back to default

    time.sleep(0.4)
    rad = mtr.read_motor_pos_rad()
    print(f"Radians travelled = {rad:.2f}\n")
    time.sleep(0.1)

    # Example (disabled): set absolute motor position
    # mtr.set_motor_pos_rad(-30.0)
    # print("set minus radians = -30.00\n")
    # time.sleep(0.1)
    # rad2 = mtr.read_motor_pos_rad()
    # print(f"stored radians = {rad2:.2f}\n")

    time.sleep(2)
    mtr.go_reverse()
    time.sleep(2)
    mtr.stop()


if __name__ == "__main__":
    main()
