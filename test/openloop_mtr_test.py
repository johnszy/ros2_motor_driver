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


# ----------------------------- STAT0 bit masks -----------------------------

STAT0_RUN_BIT      = 0
STAT0_DIR_BIT      = 1
STAT0_MODE_BIT     = 2   # 0=open-loop, 1=closed-loop
STAT0_CONFIG_BIT   = 3   # 1=config mode

STAT0_RUN_MASK     = 1 << STAT0_RUN_BIT
STAT0_DIR_MASK     = 1 << STAT0_DIR_BIT
STAT0_MODE_MASK    = 1 << STAT0_MODE_BIT
STAT0_CONFIG_MASK  = 1 << STAT0_CONFIG_BIT


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

    # ---- Read/Modify/Write STAT0 safely ----
    def _stat0_rmw(self, *, set_mask: int = 0, clear_mask: int = 0) -> int:
        """
        Read STAT0, apply set/clear masks, write back, return new value.
        """
        s = self.read_stat0()
        s = (s | (set_mask & 0xFF)) & (~clear_mask & 0xFF)
        self.write_stat0(s)
        return s

    # ---- Mode helpers ----
    def set_open_loop(self) -> int:
        # MODE=0
        return self._stat0_rmw(clear_mask=STAT0_MODE_MASK)

    def set_closed_loop(self) -> int:
        # MODE=1
        return self._stat0_rmw(set_mask=STAT0_MODE_MASK)

    def enter_config(self) -> int:
        # CONFIG=1, and typically RUN=0 while configuring
        return self._stat0_rmw(set_mask=STAT0_CONFIG_MASK, clear_mask=STAT0_RUN_MASK)

    def exit_config(self) -> int:
        # CONFIG=0
        return self._stat0_rmw(clear_mask=STAT0_CONFIG_MASK)

    # ---- Direction + run helpers (don’t clobber MODE/CONFIG) ----
    def set_forward(self) -> int:
        return self._stat0_rmw(clear_mask=STAT0_DIR_MASK)

    def set_reverse(self) -> int:
        return self._stat0_rmw(set_mask=STAT0_DIR_MASK)

    def start(self) -> int:
        return self._stat0_rmw(set_mask=STAT0_RUN_MASK)

    def stop(self) -> int:
        return self._stat0_rmw(clear_mask=STAT0_RUN_MASK)

    # ---- One-shot convenience actions ----
    def go_forward(self):
        self.set_forward()
        time.sleep(.2)
        self.start()
        print("Motor FORWARD + RUN")

    def go_reverse(self):
        self.set_reverse()
        self.start()
        print("Motor REVERSE + RUN")

    def go_config(self):
        self.enter_config()
        print("Motor CONFIG mode entered")

    # ---- Config-only writes wrapped safely ----
    def configure_pid_and_tpr(self, *, kp: int = None, ki: int = None, kd: int = None, tpr: int = None):
        """
        Enter CONFIG, write selected params, exit CONFIG.
        """
        self.enter_config()
        time.sleep(0.05)  # small settle time (safe with your PIC loop)

        if kp is not None: self.write_kp(kp)
        if ki is not None: self.write_ki(ki)
        if kd is not None: self.write_kd(kd)
        if tpr is not None: self.write_tpr(tpr)

        time.sleep(0.05)
        self.exit_config()

    # ---- Closed-loop start helper ----
    def start_closed_loop(self, *, target_rpm: int, direction: str = "fwd"):
        """
        Set MODE=CLOSED_LOOP, set target RPM, then RUN.
        """
        self.stop()
        self.exit_config()          # ensure not stuck in config
        #self.set_closed_loop()
        self.write_target_rpm(target_rpm)
        time.sleep(.2)
        self.write_stat0(5)
        time.sleep(.2)
        if direction.lower().startswith("r"):
            self.set_reverse()
        else:
            self.set_forward()

        self.start()

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
    
def _print_stat0_decode(stat0: int):
    run    = 1 if (stat0 & STAT0_RUN_MASK) else 0
    direc  = 1 if (stat0 & STAT0_DIR_MASK) else 0
    mode   = 1 if (stat0 & STAT0_MODE_MASK) else 0
    cfg    = 1 if (stat0 & STAT0_CONFIG_MASK) else 0
    print(f"STAT0=0x{stat0:02X}  RUN={run} DIR={direc} MODE={'CL' if mode else 'OL'} CFG={cfg}")


def test_closed_loop_step_response(
    target_rpm: int = 180,
    sample_s: float = 0.25,
    duration_s: float = 6.0,
    settle_window_s: float = 2.0,
    allowed_error_rpm: int = 40,
    direction: str = "fwd",
):
    ft = FT232H()
    mtr = MotorCtrl(ft, I2C_ADDRESS_DEFAULT)

    print("I2C addresses found:", [hex(a) for a in ft.scan()])

    # Ensure known config (optional: set gains/tpr here)
    # mtr.configure_pid_and_tpr(kp=5500, ki=1200, kd=50, tpr=205)

    mtr.start_closed_loop(target_rpm=target_rpm, direction=direction)

    t0 = time.time()
    samples = []
    while True:
        t = time.time() - t0
        rpm = mtr.read_meas_rpm()
        stat0 = mtr.read_stat0()
        samples.append((t, rpm, stat0))
        print(f"t={t:5.2f}s  rpm={rpm:6d}", end="  ")
        _print_stat0_decode(stat0)
        if t >= duration_s:
            break
        time.sleep(sample_s)

    mtr.stop()

    # Evaluate “settled” error over the last settle_window_s
    t_end = samples[-1][0]
    tail = [rpm for (t, rpm, _) in samples if t >= (t_end - settle_window_s)]
    if not tail:
        print("Not enough samples to evaluate settling.")
        return

    avg_tail = sum(tail) / len(tail)
    err = avg_tail - target_rpm
    print(f"\nTail avg rpm = {avg_tail:.1f}, target = {target_rpm}, error = {err:.1f} rpm")

    if abs(err) <= allowed_error_rpm:
        print("PASS: closed-loop settles near target (within allowed error).")
    else:
        print("FAIL: closed-loop did not settle near target (consider PID/tpr, friction, supply, polarity).")


def test_closed_loop_direction_flip(target_rpm: int = 160):
    ft = FT232H()
    mtr = MotorCtrl(ft, I2C_ADDRESS_DEFAULT)

    print("I2C addresses found:", [hex(a) for a in ft.scan()])

    mtr.start_closed_loop(target_rpm=target_rpm, direction="fwd")
    time.sleep(2.0)

    print("\nFlip direction -> reverse (keeping CLOSED_LOOP + RUN via masks)")
    mtr.set_reverse()     # RMW: should NOT clear mode bit
    time.sleep(2.0)

    mtr.stop()
    print("Done.")

def test_tpr():
    ft = FT232H()
    mtr = MotorCtrl(ft, I2C_ADDRESS_DEFAULT)

    addrs = ft.scan()
    print("I2C addresses found:", [hex(a) for a in addrs])

    stat0 = mtr.read_stat0()
    print(f"MTR_STAT0 = 0x{stat0:02X}")
    print(f"TPR = {mtr.read_tpr()}")
   


    # ------ run wheel forward -----------
    mtr.write_pwm(350)
    # time.sleep(.2)
    # mtr.set_open_loop()
    # time.sleep(.2)
    # time.sleep(1)
    # mtr.go_forward()
    mtr.write_stat0(1)

    for _ in range(10):
        rad = mtr.read_motor_pos_rad()
        print(f"Radians travelled = {rad:.2f}\n")
        rpm = mtr.read_meas_rpm()
        print(f"Current rpms = {rpm}\n")
        time.sleep(0.3)

    mtr.stop()
    time.sleep(2)
    mtr.go_config()
    time.sleep(2)
    stat0 = mtr.read_stat0()
    print(f"MTR_STAT0 = 0x{stat0:02X}")
    mtr.write_tpr(21)

    time.sleep(3)
    mtr.write_stat0(0)
    print(f"TPR = {mtr.read_tpr()}")

        # ------ run wheel forward -----------
    mtr.write_pwm(350)

    mtr.go_forward()

    for _ in range(10):
        rad = mtr.read_motor_pos_rad()
        print(f"Radians travelled = {rad:.2f}\n")
        rpm = mtr.read_meas_rpm()
        print(f"Current rpms = {rpm}\n")
        time.sleep(0.3)

    mtr.stop()
    mtr.go_config()
    time.sleep(2)
    stat0 = mtr.read_stat0()
    print(f"MTR_STAT0 = 0x{stat0:02X}")
    mtr.write_tpr(205)  # back to default
    time.sleep(.1)
    stat0 = mtr.read_stat0()
    print(f"MTR_STAT0 = 0x{stat0:02X}")





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
    # Quick closed-loop step test
    test_closed_loop_step_response(target_rpm=380, direction="fwd")

    # Or direction flip test
    # test_closed_loop_direction_flip(target_rpm=160)
    
    # --------------OPEN LOOP ---------------------
    #test_tpr()
    #main()