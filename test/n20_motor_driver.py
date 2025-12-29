#import smbus2
from time import sleep      
from enum import IntEnum 

from smbus2 import SMBus


# Global Variables
GRAVITIY_MS2 = 9.80665
address = None
#bus = smbus.SMBus(1)

#n20 motor registers
class MOT_REG(IntEnum):

    MTR_STAT0 = 0 
    MTR_STAT1 = 1     

    MSB_MEAS_RPM = 2     
    LSB_MEAS_RPM = 3    

    MSB_TARGET_RPM = 4   
    LSB_TARGET_RPM = 5  

    MSB_PWM = 6          
    LSB_PWM = 7          

    MSB_KP = 8         
    LSB_KP = 9          

    MSB_KI = 10         
    LSB_KI = 11

    MSB_KD = 12      
    LSB_KD = 13        

    MSB_POSITION = 14 
    LSB_POSITION = 15

class N20_MOT_DRV:

    addr = None
    BUS_NUM = 2
    #ADDR = 0x18
    bus = SMBus(BUS_NUM)

    def __init__(self, address):
        
        self.address = address

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)



    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.addr, register)
        low = self.bus.read_byte_data(self.addr, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
        
    def convert_16bit_to_bytes(self,number_16bit):
        """
        Converts a 16-bit integer to a tuple containing its high and low bytes.

        Args:
        number_16bit: An integer between 0 and 65535 (inclusive).

        Returns:
        A tuple (high_byte, low_byte) where both are integers between 0 and 255.
        """
        if not 0 <= number_16bit <= 65535:
            raise ValueError("Number must be a valid 16-bit unsigned integer (0-65535)")

        # Right shift by 8 bits to get the high byte
        high_byte = (number_16bit >> 8) & 0xFF

        # Use the bitwise AND operator to get the low byte
        low_byte = number_16bit & 0xFF

        return (high_byte, low_byte)

    def Stop(self):
        self.bus.write_byte_data(self.addr, MOT_REG.MTR_STAT0, 0x00)

    def Start(self):
        self.bus.write_byte_data(self.addr, MOT_REG.MTR_STAT0, 0x01)

    def LoadPWM(self,pwm):
        msb_byte,lsb_byte = self.convert_16bit_to_bytes(pwm)
        self.bus.write_byte_data(self.addr, MOT_REG.MSB_PWM, msb_byte)
        self.bus.write_byte_data(self.addr, MOT_REG.LSB_PWM, lsb_byte)
    

    def read_word(self,reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) | low
        if val & 0x8000:
            return -((65535 - val) + 1)
        return val

if __name__ == "__main__":

    drv = N20_MOT_DRV()
    for i in range(0,4):
        pwm = 300 + (i * 20)
        drv.LoadPWM(pwm)
        drv.Start()
        sleep(2)
        drv.Stop()
        sleep(2)



    
