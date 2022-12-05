PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47


def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    bus.write_byte_data(0x53, 0x2C, 0x0B)
    value = bus.read_byte_data(0x53, 0x31)
    value &= ~0x0F;
    value |= 0x0B;
    value |= 0x08;
    bus.write_byte_data(0x53, 0x31, value)
    bus.write_byte_data(0x53, 0x2D, 0x08)


def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if (value > 32768):
        value = value - 65536
    return value


bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()