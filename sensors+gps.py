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

#This file reads the gps and sensor values and stores them in a csv file


ts = []
lat = []
lat_d = []
lon = []
long_d = []
sp = []
mag_var = []
mag_var_d = []

mag = magneto2.QMC5883L()

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
print(" Reading Data of Gyroscope and Accelerometer")


def getAxes():
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
    print("Reading Accelerometer from ADXL")
    print("   Ax = %.3f ms2" % x)
    print("   Ay = %.3f ms2" % y)
    print("   Az = %.3f ms2" % z)
    acc_lis_x.append(x)
    acc_lis_y.append(y)
    acc_lis_z.append(z)
    return {"x": x, "y": y, "z": z}


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
sensor_data = np.transpose(
    [ts, th_lis, tm_lis, ts_lis, tms_lis, acc_lis_x, acc_lis_y, acc_lis_z, gyro_lis_x, gyro_lis_y, gyro_lis_z,
     mag_lis_x,
     mag_lis_y, mag_lis_z])
sensor = pd.DataFrame(sensor_data,
                      columns=['Timestamp', 'Hour', 'Minute', 'Second', 'MicroSecond', 'Accelerometer X',
                               'Accelerometer Y',
                               'Accelerometer Z', 'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z', 'Magnetometer X',
                               'Magnetometer Y', 'Magnetometer Z'])
sensor.to_csv('Sensor.csv')
x = pd.read_csv('Sensor.csv')
print(x)
gps_col = ['Timestamp', 'Latitude', 'Latitude Direction', 'Longitude',
           'Longitude Direction', 'Speed Over Ground',
           'Magnetic Variation', 'Magnetic Variation Direction']
gps_data = np.transpose([ts, lat, lat_d, lon, long_d, sp, mag_var, mag_var_d])
gps = pd.DataFrame(gps_data, columns=gps_col)
gps.to_csv('GPS_data.csv')
x = pd.read_csv('GPS_data.csv')
print(x)







