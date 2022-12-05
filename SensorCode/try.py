#importing the required libraries
import smbus
from time import sleep
import time
import sys
import magneto2
import numpy as np
import pandas as pd
# from datetime import timezone
import datetime
import pynmea2
import serial
import logging
import math

####Magnetometer definition########
DFLT_BUS = 1
DFLT_ADDRESS = 0x0d

REG_XOUT_LSB = 0x00
REG_XOUT_MSB = 0x01
REG_YOUT_LSB = 0x02
REG_YOUT_MSB = 0x03
REG_ZOUT_LSB = 0x04
REG_ZOUT_MSB = 0x05
REG_STATUS_1 = 0x06
REG_TOUT_LSB = 0x07
REG_TOUT_MSB = 0x08
REG_CONTROL_1 = 0x09
REG_CONTROL_2 = 0x0a
REG_RST_PERIOD = 0x0
REG_CHIP_ID = 0x0d

STAT_DRDY = 0b00000001
STAT_OVL = 0b00000010
STAT_DOR = 0b00000100

INT_ENB = 0b00000001
POL_PNT = 0b01000000
SOFT_RST = 0b10000000

MODE_STBY = 0b00000000
MODE_CONT = 0b00000001
ODR_10HZ = 0b00000000
ODR_50HZ = 0b00000100
ODR_100HZ = 0b00001000
ODR_200HZ = 0b00001100
RNG_2G = 0b00000000
RNG_8G = 0b00010000
OSR_512 = 0b00000000
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000

class QMC5883L(object):
    def __init__(self,
                 i2c_bus=DFLT_BUS,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_10HZ,
                 output_range=RNG_2G,
                 oversampling_rate=OSR_512):

        self.address = address
        self.bus = smbus.SMBus(i2c_bus)
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

    def __del__(self):
        self.mode_standby()

    def mode_continuous(self):
        self._write_byte(REG_CONTROL_2, SOFT_RST)
        self._write_byte(REG_CONTROL_2, INT_ENB)
        self._write_byte(REG_RST_PERIOD, 0x01)
        self._write_byte(REG_CONTROL_1, self.mode_cont)

    def mode_standby(self):
        self._write_byte(REG_CONTROL_2, SOFT_RST)
        self._write_byte(REG_CONTROL_2, INT_ENB)
        self._write_byte(REG_RST_PERIOD, 0x01)
        self._write_byte(REG_CONTROL_1, self.mode_stby)

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        time.sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(self.address, registry)

    def _read_word(self, registry):
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        val = self._read_word(registry)
        if val >= 0x8000:
            return val - 0x10000
        else:
            return val

    def get_data(self):
        i = 0
        [x, y, z, t] = [None, None, None, None]
        while i < 20:
            status = self._read_byte(REG_STATUS_1)
            if status & STAT_OVL:
                msg = ("Magnetic sensor overflow.")
                if self.output_range == RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                logging.warning(msg)
            if status & STAT_DOR:
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                continue
            if status & STAT_DRDY:
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                t = self._read_word_2c(REG_TOUT_LSB)
                break
            else:
                time.sleep(0.01)
                i += 1
        return [x, y, z, t]

    def get_magnet_raw(self):
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + z*c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + z*c[1][2]
            z1 = x * c[2][0] + y * c[2][1] + z*c[2][2]
        return [x1, y1,z1]

    def get_bearing_raw(self):
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b

    def get_temp(self):
        [x, y, z, t] = self.get_data()
        return t

    def set_declination(self, value):
            try:
             d = float(value)
             if d < -180.0 or d > 180.0:
                logging.error(u'Declination must be >= -180 and <= 180.')
             else:
                self._declination = d
            except:
             logging.error(u'Declination must be a float value.')

    def get_declination(self):
        return self._declination

    def set_calibration(self, value):
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            logging.error(u'Calibration must be a 3x3 float matrix.')

    def get_calibration(self):
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')
mag = QMC5883L()


####Gyroscope definition##########
###SDA->24,SCL->23
print(" Reading Data of Gyroscope ")
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
    value &= ~0x0F
    value |= 0x0B
    value |= 0x08
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

######Accelerometer Definition##########
def getAxes():
    print("Reading Accelerometer from ADXL")
    bytes = bus.read_i2c_block_data(0x53, 0x32, 6)
    x = bytes[0] | (bytes[1] << 8)
    if (x & (1 << 16 - 1)):
        x = x - (1 << 16)
    y = bytes[2] | (bytes[3] << 8)
    if (y & (1 << 16 - 1)):
        y = y - (1 << 16)
    z = bytes[4] | (bytes[5] << 8)
    if (z & (1 << 16 - 1)):
        z = z - (1 << 16)
    x = x * 0.004
    y = y * 0.004
    z = z * 0.004
    x = x * 9.80665
    y = y * 9.80665
    z = z * 9.80665
    x = round(x, 4)
    y = round(y, 4)
    z = round(z, 4)
    print("   Ax = %.3f ms2" % x)
    print("   Ay = %.3f ms2" % y)
    print("   Az = %.3f ms2" % z)
    acc_lis_x.append(x)
    acc_lis_y.append(y)
    acc_lis_z.append(z)
    return {"x": x, "y": y, "z": z}

########List Definitions############
ts = []
lat = []
lat_d = []
lon = []
long_d = []
sp = []
mag_var = []
mag_var_d = []
mag_lis_x = []
mag_lis_y = []
mag_lis_z = []
acc_lis_x = []
acc_lis_y = []
acc_lis_z = []
gyro_lis_x = []
gyro_lis_y = []
gyro_lis_z = []
# date_lis=[]
th_lis = []
tm_lis = []
ts_lis = []
tms_lis = []

#######Loop###########
port = "/dev/ttyAMA0"

try:
    while True:

        dataout = pynmea2.NMEAStreamReader()
        ser = serial.Serial(port, baudrate=10000, timeout=0.25)
        newdata = ser.readline()
        # print(newdata)
        if newdata[0:6] == "$GNRMC":
            newmsg = pynmea2.parse(newdata)
            la = newmsg.latitude
            ln = newmsg.longitude
            # alt = newmsg. altitude
            nmeaobj = newmsg
            print((nmeaobj.fields[0][0], nmeaobj.data[0]))
            l = ['%s: %s' % (nmeaobj.fields[i][0], nmeaobj.data[i])
                 for i in range(len(nmeaobj.fields))]
            print(l)
            ts.append(nmeaobj.data[0])
            lat.append(la)
            lon.append(ln)
            lat_d.append(nmeaobj.data[3])
            long_d.append(nmeaobj.data[5])
            sp.append(nmeaobj.data[6])
            mag_var.append(nmeaobj.data[9])
            mag_var_d.append(nmeaobj.data[10])
            getAxes()
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_YOUT_H)
            gyro_z = read_raw_data(GYRO_ZOUT_H)

            gyro_lis_x.append(gyro_x)
            gyro_lis_y.append(gyro_y)
            gyro_lis_z.append(gyro_z)

            Ax = acc_x * 9.806 / 16384.0
            Ay = acc_y / 16384.0
            Az = acc_z / 16384.0
            Gx = gyro_x / 131.0
            Gy = gyro_y / 131.0
            Gz = gyro_z / 131.0
            print(" Reading Data of Gyroscope from mpu")
            print("   Gx = %.3f " % Gx)
            print("   Gy = %.3f " % Gy)
            print("   Gz = %.3f " % Gz)
            m = mag.get_magnet()
            print(" Reading Data of Magnetometer")
            print("   Mx = %.3f " % m[0])
            print("   My = %.3f " % m[1])
            print("   Mz = %.3f " % m[2])
            print("################################")
            mag_lis_x.append(m[0])
            mag_lis_y.append(m[1])
            mag_lis_z.append(m[2])
            dt = datetime.datetime.now()
            th_lis.append(dt.hour)
            tm_lis.append(dt.minute)
            ts_lis.append(dt.second)
            tms_lis.append(dt.microsecond)

except KeyboardInterrupt:
    print("Stored in csv")
    sensor_data = np.transpose([ts, th_lis, tm_lis, ts_lis, tms_lis, acc_lis_x, acc_lis_y, acc_lis_z, gyro_lis_x, gyro_lis_y, gyro_lis_z,mag_lis_x,mag_lis_y, mag_lis_z])
    sensor = pd.DataFrame(sensor_data,columns=['Timestamp', 'Hour', 'Minute', 'Second', 'MicroSecond', 'Accelerometer X',
                                   'Accelerometer Y','Accelerometer Z', 'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z',
                                   'Magnetometer X','Magnetometer Y', 'Magnetometer Z'])
    sensor.to_csv('Sensor.csv')
    s = pd.read_csv('Sensor.csv')
    print(s)
    gps_col = ['Timestamp', 'Latitude', 'Latitude Direction', 'Longitude','Longitude Direction', 'Speed Over Ground',
               'Magnetic Variation', 'Magnetic Variation Direction']
    gps_data = np.transpose([ts, lat, lat_d, lon, long_d, sp, mag_var, mag_var_d])
    gps = pd.DataFrame(gps_data, columns=gps_col)
    gps.to_csv('GPS_data.csv')
    g = pd.read_csv('GPS_data.csv')
    print(g)
