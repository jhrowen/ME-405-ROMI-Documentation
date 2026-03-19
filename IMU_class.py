"""
@file IMU_class.py
@brief File containing class definition for IMU sensor object
@details 
IMU object class with various methods. Includes methods to initialize, change mode,
and read euler angles, angular velocity and acceleration
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
import pyb
from pyb import I2C
from micropython import const
from time import sleep_ms
import os


## @name I2C address options
## @{
ADDR_A = const(0x28) ## @brief Default address for BNO055
ADDR_B = const(0x29) ## @brief Alternate address
## @}

# --- Registers (subset) ---
## @name Registers
## @{
CHIP_ID     = const(0x00) ## @brief Chip ID
PAGE_ID     = const(0x07) ## @brief Page ID

ACC_DATA_X_LSB   = const(0x08) ## @brief Accelerometer X LSB
MAG_DATA_X_LSB   = const(0x0E) ## @brief Magnetometer X LSB
GYR_DATA_X_LSB   = const(0x14) ## @brief Gyroscope X LSB
EULER_H_LSB      = const(0x1A) ## @brief Euler Angles LSB
QUA_W_LSB        = const(0x20) ## @brief Quaternions LSB
LIA_DATA_X_LSB   = const(0x28) ## @brief Linear Acceleration X LSB
GRV_DATA_X_LSB   = const(0x2E) ## @brief Gravity X LSB

ACC_OFF_X_LSB = const(0x55) ## @brief Accelerometer offset LSB
MAG_OFF_X_LSB = const(0x5B) ## @brief Magnetometer offset LSB
GYR_OFF_X_LSB = const(0x61) ## @brief Gyroscope offset LSB
ACC_RAD_LSB = const(0x67) ## @brief Acceleration Radius LSB

CALIB_STAT  = const(0x35) ## @brief Calibration status byte

## @}

OPR_MODE    = const(0x3D)
PWR_MODE    = const(0x3E)
SYS_TRIGGER = const(0x3F)
UNIT_SEL    = const(0x3B)

# --- Operation modes ---
## @name Operational Modes
MODE_CONFIG = const(0x00) ## @brief Configuration Mode
MODE_NDOF   = const(0x0C) ## @brief NDOF Fusion mode
MODE_IMU    = const(0x08) ## @brief IMU Mode
## @}

# --- Power modes ---
POWER_NORMAL = const(0x00)

## @brief BNO055 Class Driver
class IMU:
    ## @brief constructor for IMU Object
    ## @param I2C I2C bus object
    ## @param addr I2C address of the BNO055
    def __init__(self, i2c, addr=ADDR_A):
        # Using I2C2, device address is 0x28
        self._i2c = i2c
        self._addr = addr
        self._buf = bytearray(8)

    ## @brief Read one byte from a designated register
    ## @param reg Register to read from
    ## @return Contents of register
    def readbyte(self, reg):
        return self._i2c.mem_read(1, self._addr, reg)[0]
    ## @brief read n bytes started at a designated register
    ## @param n number of bytes to read
    ## @param reg register to start reading from
    ## @return Contents of n registers
    def readbytes(self, n, reg):
        return self._i2c.mem_read(n, self._addr, reg)
    ## @brief write a byte to a register 
    ## @param val Value to write
    ## @param reg Register to write to
    def writebytes(self, val, reg):
        self._i2c.mem_write(bytes([val & 0xFF]), self._addr, reg)
    ## @brief write 2 bytes to a register
    ## @param val Value to write
    ## @param Register to write to
    def write2bytes(self, val, reg):
        val &= 0xFFFF
        lsb = val&0xFF
        msb = (val >> 8) & 0xFF
        self.writebytes(lsb,reg)
        self.writebytes(msb,reg+1)
    ## @brief combine MSB and LSB into 16 bit number
    ## @param lsb Contents of least significant bit
    ## @param msb Contents of most significant bit
    ## @return 16 bit number
    @staticmethod
    def combine(lsb, msb):
        v = (msb<<8) | lsb
        return v-65536 if v & 0x8000 else v
    ## @brief Read an x,y and z data starting from the X LSB
    ## @param startreg Register to start reading from
    ## @param unit_scale Unit scale adjustment based on what vector is being read
    ## @return Tuple containing 16 bit number x, y and z
    def read_vector(self, startreg, unit_scale):
        data = self.readbytes(6, startreg)
        x = self.combine(data[0], data[1]) / unit_scale
        y = self.combine(data[2], data[3]) / unit_scale
        z = self.combine(data[4], data[5]) / unit_scale
        return x,y,z
    ## @brief Change the Mode the IMU is configured in
    ## @param mode Mode to configure the IMU in
    def set_mode(self, mode):
        self.writebytes(mode, OPR_MODE)
        sleep_ms(30)
    ## @brief Initialize 
    ## @param mode Mode to initialize IMU into
    ## @param reset
    ## @param units Output units from BNO055
    def initialize(self, mode=MODE_IMU, reset=True, units = 0x06):
        # units 0x00 gives m/s^2, degrees/sec, Celcius
        self.writebytes(0x00, PAGE_ID)
        sleep_ms(5)
        self.set_mode(MODE_CONFIG)
        # Power Mode: Normal
        self.writebytes(POWER_NORMAL, PWR_MODE)
        sleep_ms(20)
        self.writebytes(units, UNIT_SEL) # select standard units
        sleep_ms(10)
        # Calibration data
        fname = "calibration.txt"
        try:
            os.stat(fname)
            with open(fname, 'r') as file:
                constants = file.read().strip().split(',')
                vals = [int(c) for c in constants if c!=""]
                print(len(vals))
                ACC = vals[0:3]
                MAG = vals[3:6]
                GYR = vals[6:9]
                print(vals)
                ACC_RAD = vals[9]
                MAG_RAD = vals[10]
                self.write_coeff(ACC, MAG, GYR, ACC_RAD, MAG_RAD)
        except OSError:
                self.set_mode(MODE_NDOF)
                while(True):
                    stat = self.calibration_status()
                    print(stat)
                    if all(bit == 3 for bit in stat[1:]):
                        print("Calibration complete")
                        break
                self.set_mode(MODE_CONFIG)
                ACC_OFF, MAG_OFF, GYR_OFF, ACC_RAD, MAG_RAD = self.get_coeff()
                lst = list(ACC_OFF) + list(MAG_OFF) + list(GYR_OFF) + [ACC_RAD, MAG_RAD]
                with open(fname, 'w') as file:
                    file.write(','.join(str(c) for c in lst)+ "\n")
        self.set_mode(mode) # enter IMU mode
    ## @brief Method to check the calibtration status bit for the IMU
    ## @param reg Register for calibration status bit
    def calibration_status(self, reg=CALIB_STAT):
        # data in the order: system, gyro, accelerometer, magnetometer
        cbt = self.readbyte(reg)
        system = (cbt >> 6) & 0b11
        gyro = (cbt >> 4) & 0b11
        acc = (cbt >> 2) & 0b11
        mag = cbt & 0b11
        return [system, gyro, acc, mag]
    ## @brief Method to read calibration coefficients from the IMU
    ## @param reg Address to start reading from
    ## @return Tuple containing sensor offsets
    def get_coeff(self,reg=ACC_OFF_X_LSB):
        # format: [x data, y data, z data]
        raw = self.readbytes(22, reg)
        ACC_OFF = (self.combine(raw[0],raw[1]) & 0xFFFF,self.combine(raw[2],raw[3]) & 0xFFFF,self.combine(raw[4],raw[5]) & 0xFFFF)
        MAG_OFF = (self.combine(raw[6],raw[7]) & 0xFFFF,self.combine(raw[8],raw[9]) & 0xFFFF,self.combine(raw[10],raw[11]) & 0xFFFF)
        GYR_OFF = (self.combine(raw[12],raw[13]) & 0xFFFF,self.combine(raw[14],raw[15]) & 0xFFFF,self.combine(raw[16],raw[17]) & 0xFFFF)
        ACC_RAD = self.combine(raw[18],raw[19]) & 0xFFFF
        MAG_RAD = self.combine(raw[20],raw[21]) & 0xFFFF
        return ACC_OFF,MAG_OFF,GYR_OFF,ACC_RAD,MAG_RAD
    ## @brief Write prerecorded calibration coefficients to their respective registers
    ## @param ACC Tuple containing acclerometer offsets
    ## @param MAG Tuple containing magnetometer offsets
    ## @param GYR Tuple containing gyroscope offsets
    ## @param ACC_RAD Acceleration radius value
    ## @param MAG_RAD Magnetometer radius value
    ## @reg Address to start writing to
    def write_coeff(self, ACC, MAG, GYR, ACC_RAD, MAG_RAD, reg=ACC_OFF_X_LSB):
        self.write2bytes(ACC[0], reg)
        self.write2bytes(ACC[1], reg+2)
        self.write2bytes(ACC[2], reg+4)
        self.write2bytes(MAG[0], reg+6)
        self.write2bytes(MAG[1], reg+8)
        self.write2bytes(MAG[2], reg+10)
        self.write2bytes(GYR[0], reg+12)
        self.write2bytes(GYR[1], reg+14)
        self.write2bytes(GYR[2], reg+16)
        self.write2bytes(ACC_RAD, reg+18)
        self.write2bytes(MAG_RAD, reg+20)
    ## @brief Read euler angles
    ## @param reg Address containing euler angles LSB
    ## @return Tuple containing euler angles in Rads
    def euler(self, reg=EULER_H_LSB):
        angles = self.read_vector(reg, 900.0)
        return angles
    ## @brief Read angular velocities
    ## @param reg Address containing angular velocities LSB
    ## @return Tuple containing angular velocities in Rads/s
    def ang_v(self, reg=GYR_DATA_X_LSB):
        rates = self.read_vector(reg, 900.0)
        return rates
    ## @brief Read acceleration data
    ## @param reg Address containing acceleration X LSB
    ## @return Tuple containing x, y and z accleration
    def accel(self, reg=ACC_DATA_X_LSB):
        accel = self.read_vector(reg, 100)
        return(accel)
