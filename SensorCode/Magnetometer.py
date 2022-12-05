import logging
import math
import time
import smbus

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
