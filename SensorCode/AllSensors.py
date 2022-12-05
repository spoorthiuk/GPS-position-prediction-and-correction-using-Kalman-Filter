import time
import board
import busio
import adafruit_adxl34x
import smbus2
from time import sleep
import logging
import math

##########mpu###########################
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


def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if (value > 32768):
        value = value - 65536
    return value


bus = smbus2.SMBus(1)
Device_Address = 0x68
MPU_Init()
################Accelerometer###################
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)
#print(" Reading Data of Gyroscope and Accelerometer")
###############Magnetometer######################
DFLT_BUS = 1
DFLT_ADDRESS = 0x0d

REG_XOUT_LSB = 0x00     # Output Data Registers for magnetic sensor.
REG_XOUT_MSB = 0x01
REG_YOUT_LSB = 0x02
REG_YOUT_MSB = 0x03
REG_ZOUT_LSB = 0x04
REG_ZOUT_MSB = 0x05
REG_STATUS_1 = 0x06     # Status Register.
REG_TOUT_LSB = 0x07     # Output Data Registers for temperature.
REG_TOUT_MSB = 0x08
REG_CONTROL_1 = 0x09    # Control Register #1.
REG_CONTROL_2 = 0x0a    # Control Register #2.
REG_RST_PERIOD = 0x0b   # SET/RESET Period Register.
REG_CHIP_ID = 0x0d      # Chip ID register.

# Flags for Status Register #1.
STAT_DRDY = 0b00000001  # Data Ready.
STAT_OVL = 0b00000010   # Overflow flag.
STAT_DOR = 0b00000100   # Data skipped for reading.

# Flags for Status Register #2.
INT_ENB = 0b00000001    # Interrupt Pin Enabling.
POL_PNT = 0b01000000    # Pointer Roll-over.
SOFT_RST = 0b10000000   # Soft Reset.

# Flags for Control Register 1.
MODE_STBY = 0b00000000  # Standby mode.
MODE_CONT = 0b00000001  # Continuous read mode.
ODR_10HZ = 0b00000000   # Output Data Rate Hz.
ODR_50HZ = 0b00000100
ODR_100HZ = 0b00001000
ODR_200HZ = 0b00001100
RNG_2G = 0b00000000     # Range 2 Gauss: for magnetic-clean environments.
RNG_8G = 0b00010000     # Range 8 Gauss: for strong magnetic fields.
OSR_512 = 0b00000000    # Over Sample Rate 512: less noise, more power.
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000     # Over Sample Rate 64: more noise, less power.


class QMC5883L(object):
    """Interface for the QMC5883l 3-Axis Magnetic Sensor."""

    def __init__(self,
                 i2c_bus=DFLT_BUS,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_10HZ,
                 output_range=RNG_2G,
                 oversampling_rate=OSR_512):

        self.address = address
        self.bus = smbus2.SMBus(i2c_bus)
        self.output_range = output_range
        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        chip_id = self._read_byte(REG_CHIP_ID)
        if chip_id != 0xff:
            msg = "Chip ID returned 0x%x instead of 0xff; is the wrong chip?"
            logging.warning(msg, chip_id)
        self.mode_cont = (MODE_CONT | output_data_rate | output_range
                          | oversampling_rate)
        self.mode_stby = (MODE_STBY | ODR_10HZ | RNG_2G | OSR_64)
        self.mode_continuous()

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        time.sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(self.address, registry)

    def mode_continuous(self):
        """Set the device in continuous read mode."""
        self._write_byte(REG_CONTROL_2, SOFT_RST)  # Soft reset.
        self._write_byte(REG_CONTROL_2, INT_ENB)  # Disable interrupt.
        self._write_byte(REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
        self._write_byte(REG_CONTROL_1, self.mode_cont)  # Set operation mode.

    def get_data(self):
        """Read data from magnetic and temperature data registers."""
        i = 0
        [x, y, z, t] = [None, None, None, None]
        while i < 20:  # Timeout after about 0.20 seconds.
            status = self._read_byte(REG_STATUS_1)
            if status & STAT_OVL:
                # Some values have reached an overflow.
                msg = ("Magnetic sensor overflow.")
                if self.output_range == RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                logging.warning(msg)
            if status & STAT_DOR:
                # Previous measure was read partially, sensor in Data Lock.
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                continue
            if status & STAT_DRDY:
                # Data is ready to read.
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                t = self._read_word_2c(REG_TOUT_LSB)
                break
            else:
                # Waiting for DRDY.
                time.sleep(0.01)
                i += 1
        return [x, y, z, t]

    def _read_word(self, registry):
        """Read a two bytes value stored as LSB and MSB."""
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        """Calculate the 2's complement of a two bytes value."""
        val = self._read_word(registry)
        if val >= 0x8000:  # 32768
            return val - 0x10000  # 65536
        else:
            return val

    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None or z is None:
            [x1, y1, z1] = [x, y, z]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + z * c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + z * c[1][2]
            z1 = x * c[2][0] + y * c[2][1] + z * c[2][2]
        return [x1, y1, z1]

sensor = QMC5883L()

while True:
    ####MPU######
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    mpu=[Ax,Ay,Az,Gx,Gy,Gz]
    ################Accelerometer###################
    acc=accelerometer.acceleration
    ###############Magnetometer######################
    m = sensor.get_magnet()

    sleep(1)
