from ch347_i2c_motor import CH347I2C

dev = CH347I2C()

I2C_ADDR = 0x14   # PIC16F1619 slave address (7-bit)

# Read P gain (reg 0x04)
p_gain = dev.read_u16(I2C_ADDR, 0x04)
print("P gain =", p_gain)

# Write I gain (reg 0x05)
dev.write_u16(I2C_ADDR, 0x05, 1234)
print("Wrote I gain = 1234")

# Read actual RPM (reg 0x01)
rpm = dev.read_u16(I2C_ADDR, 0x01)
print("Measured RPM =", rpm)

dev.close()
