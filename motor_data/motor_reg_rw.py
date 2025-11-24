#! /usr/bin/env python
#coding=utf-8
import os
import time
from ctypes import *

class USBI2C():
    ch347 = windll.LoadLibrary("./CH347DLLA64.dll")
    def __init__(self, usb_dev = 0, i2c_dev = 0x14):
        self.usb_id   = usb_dev
        self.dev_addr = i2c_dev
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) != -1:
            USBI2C.ch347.CH347CloseDevice(self.usb_id)
        else:
            print("USB CH347 Open Failed!")

    def read_reg(self, reg_addr, size):
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) != -1:
            
            # First byte = device address + register address
            tcmd = (c_byte * 2)()
            tcmd[0] = self.dev_addr
            tcmd[1] = reg_addr & 0xFF

            # Buffer for reading
            ibuf = (c_byte * size)()

            # Perform write/read combined transaction
            USBI2C.ch347.CH347StreamI2C(self.usb_id, 
                                        2,    # write length
                                        tcmd, # write buffer
                                        size, # read length
                                        ibuf) # read buffer

            USBI2C.ch347.CH347CloseDevice(self.usb_id)
            return ibuf
        else:
            print("USB CH347 Open Failed!")
            return None

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
            0,
            write_buf,
            1,     # bytes to read
            read_buf
        )

        USBI2C.ch347.CH347CloseDevice(self.usb_id)

        print("Stream result:", res)
        print("Raw read:", [hex(read_buf[i] & 0xFF) for i in range(2)])

        return ((read_buf[0] & 0xFF) << 8) | (read_buf[1] & 0xFF)




    def read(self):
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) != -1:
            
            rec  = (c_byte * 1)()
            ibuf = (c_byte * 9)()
            rec[0] = self.dev_addr
            
            USBI2C.ch347.CH347StreamI2C(self.usb_id, 1, rec, 9, ibuf)
            USBI2C.ch347.CH347CloseDevice(self.usb_id)
            
            return ibuf
        else:
            print("USB CH347 Open Failed!")
            return 0

    def write(self,cmd,size):
        if USBI2C.ch347.CH347OpenDevice(self.usb_id) != -1:
            tcmd = (c_byte * (size + 1))()
            ibuf = (c_byte * 1)()
            tcmd[0] = self.dev_addr
            
            for i in range (size):
                tcmd[i+1] = cmd[i] & 0xff

            USBI2C.ch347.CH347StreamI2C(self.usb_id, 6, tcmd, 0, ibuf)
            USBI2C.ch347.CH347CloseDevice(self.usb_id)
        else:
            print("USB CH347 Open Failed!")

if __name__ == "__main__":
    # cmd = (c_byte * 5)(0x5a,0x05,0x00,0x01,0x60)
    # size = sizeof(cmd) 
    # q = USBI2C()
    # while True:
    #     q.write(cmd,size)
    #     rec =q.read()
    #     dist    =((rec[2]&0xff)+(rec[3]&0xff)*256)
    #     strengh =((rec[4]&0xff)+(rec[5]&0xff)*256)
    #     temp    =((rec[6]&0xff)+(rec[7]&0xff)*256)/8-256
    #     print("Dist:",dist,"Strengh:",strengh,"Temp:",temp)
  
    #     time.sleep(0.05) #50ms
    q = USBI2C()

    val = q.read_reg(4, 1)   # read 1 byte from register 4
    print("motor_reg[4] =", val[0] & 0xFF)

    regs = q.read_reg(0, 8)  # read motor_reg[0..7]
    print([regs[i] & 0xFF for i in range(8)])
    
    # q = USBI2C(i2c_dev=0x14)

    rpm = q.read_reg16(0)       # motor_regs[0]
    temp = q.read_reg16(1)      # motor_regs[1]
    error = q.read_reg16(2)     # motor_regs[2]
    kp = q.read_reg16(4)       # motor_regs[0]
    ki = q.read_reg16(5)      # motor_regs[1]
    kd = q.read_reg16(6)     # motor_regs[2]

    print("rpm =", rpm)
    print("temp =", temp)
    print("error =", error)
    print("kp =", kp)
    print("ki =", ki)
    print("kd =", kd)