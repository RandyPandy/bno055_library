##-*- coding: utf-8 -*-
"""
@file bno055.py
This file contains a MicroPython driver for the bno055 accelerometer. 

@author Randy
"""

import micropython


## The register address of the STATUS register in the MMA845x
STATUS_REG = micropython.const (0x39)

## The register address of the OUT_X_MSB register in the MMA845x
OUT_X_MSB = micropython.const (0x09)

## The register address of the OUT_X_LSB register in the MMA845x
OUT_X_LSB = micropython.const (0x08)

## The register address of the OUT_Y_MSB register in the MMA845x
OUT_Y_MSB = micropython.const (0x0B)

## The register address of the OUT_Y_LSB register in the MMA845x
OUT_Y_LSB = micropython.const (0x0A)

## The register address of the OUT_Z_MSB register in the MMA845x
OUT_Z_MSB = micropython.const (0x0D)

## The register address of the OUT_Z_LSB register in the MMA845x
OUT_Z_LSB = micropython.const (0x0C)

## The register address of the WHO_AM_I register in the MMA845x
WHO_AM_I = micropython.const (0x00)

## The register address of the DATA_CFG_REG register in the MMA845x which is
#  used to set the measurement range to +/-2g, +/-4g, or +/-8g
XYZ_DATA_CFG = micropython.const (0x0E)

## OPR MODE
OPR_MODE_CFG = 0x3D
OPR_MODE_NDOF = int('00001100',2)
OPR_MODE_IMU = int('00001000',2)

##Accel Range Reg


##Unit select UNIT_SEL xxxxxxx0b
UNIT_SEL_REG = 0x3B

#default unit select.  windows, celsius, degrees, dps, m/s
UNIT_SELECT = 0x80

#Acceleration xxxxx0b
linear = int('00000000',2) # linear m/s 
gravity = int('00000001',2) #Gravity vector mg

##Angle Rate xxxxx0xb
degrees = int('00000000',2) #degrees/s
radians = int('00000010',2) #rad/s

#Euler angles xxxx0xxb
euler_degree = int('00000000',2)
euler_rad = int('00000100',2)

#Temperature 
celsius = ('00000000',2)
fahrenheit = int('00010000',2)

#Fusion data output format 0xxxxxxxb
'''Roll: -90° to +90° (increasing with increasing inclination)
Yaw: 0° to 360° (turning clockwise increases values)'''
windows = int('00000000',2) # Pitch -180° to +180° (turing clock-wise increases values
android = int('10000000',2) # Pitch +180° to -180° (turning clockwise decreases values)

## Eul_Roll MSB 
OUT_EUL_ROLL_MSB = 0x1D

## Eul_Roll LSB
OUT_EUL_ROLL_LSB = 0x1C

## Eul_Pitch MSB
OUT_EUL_PITCH_MSB = 0x1F

## Eul_PITCH LSB
OUT_EUL_PITCH_LSB = 0x1E

## EUL_HEADING MSB
OUT_EUL_HEADING_MSB = 0x1B 

## EUL_HEADING_LSB
OUT_EUL_HEADING_LSB = 0x1A


class BNO055:
    """ This class implements a simple driver for bno055 accelerometer and includes some functions to read accel data """

    def __init__ (self, i2c, address, accel_range = 0):
        """ Initialize an MMA845x driver on the given I<sup>2</sup>C bus. The 
        I<sup>2</sup>C bus object must have already been initialized, as we're
        going to use it to get the accelerometer's WHO_AM_I code right away. 
        @param i2c An I<sup>2</sup>C bus already set up in MicroPython
        @param address The address of the accelerometer on the I<sup>2</sup>C
            bus 
        @param accel_range The range of accelerations to measure; it must be
            either @c RANGE_2g, @c RANGE_4g, or @c RANGE_8g (default: 2g)
        """

        ## The I2C driver which was created by the code which called this
        self.i2c = i2c

        ## The I2C address at which the accelerometer is located
        self.addr = address
        self.i2c.mem_write(0x18, 40, 0x3D)
        
    def set_accel(self,select):
        '''Unit select for acceleration'''
        ## @param select, 0 for linear accel m/s, 1 for gravity vector mg
        if select==1:
            unit = linear|UNIT_SELECT
            print("accel has been set to m/s")
        if select==0:
            unit = gravity|UNIT_SELECT
        
            print("accel has been set to mg")

    def get_ax_bits (self):
        """ Get the X acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured X acceleration in A/D conversion bits """
        a = self.i2c.mem_read(2, self.addr, OUT_X_LSB)
        ax = ((a[1]<<8) + a[0])/16.0
        print ('ax',ax)
        return 0


    def get_ay_bits (self):
        """ Get the Y acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Y acceleration in A/D conversion bits """
        y = self.i2c.mem_read(2, self.addr, OUT_Y_LSB)
        ay = ((y[1]<<8) + y[0])/16.0
        print ('ay',ay)
        return 0


    def get_az_bits (self):
        """ Get the Z acceleration from the accelerometer in A/D bits and 
        return it.
        @return The measured Z acceleration in A/D conversion bits """
        z = self.i2c.mem_read(2, self.addr, OUT_Z_LSB)
        az = ((z[1]<<8) + z[0])/100.0
        print ('az',az)
        return 0

    def get_accels (self):
        """ Get all three accelerations from the bno055 """
        return (self.get_ax (), self.get_ay (), self.get_az ())

    def get_EUL_Heading(self):
        """Orientation output only available in fusion operation modes.The  fusion  
        algorithm  output  offset  and  tilt  compensated  orientation  data  in 
        Eulerangles format for each DOF Heading/Roll/Pitch, the output data can be 
        read from the  appropriate EUL<dof>_ LSB and EUL_ <dof>_ MSB registers. Refer 
        table below for information regarding the data types and the unit 
        representation for the Euler angle format. Signed 2 Bytes. 1 Degree = 16LSB.
        1 RAD = 900 LSB"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_HEADING_LSB)
        eul_heading = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return eul_heading
        #print(eul_heading)
    
    def get_EUL_Roll(self):
        """Returns EUL roll bytes. Signed 2 Bytes"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_ROLL_LSB)
        eul_roll = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return eul_roll
        #print(eul_roll)
    
    def get_EUL_Pitch(self):
        """Returns EUL Pitch bytes. Signed 2 Bytes"""
        eul_raw = self.i2c.mem_read(2, self.addr, OUT_EUL_PITCH_LSB)
        eul_pitch = self.sign_val(((eul_raw[1]<<8) + eul_raw[0]))/16.0
        return (eul_pitch)
        #print(eul_pitch)


    def get_eul(self):
        roll = self.get_EUL_Roll()
        heading = self.get_EUL_Heading()
        pitch = self.get_EUL_Pitch()
        print(' roll',roll,'\n','heading',heading,'\n','pitch',pitch,'\n')

    def sign_val(self,value):
        """converts number to signed number"""
        if value >= 0x8000:
            value -= 0x10000
        return value

