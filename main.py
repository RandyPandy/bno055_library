#
# micropython i2c driver
#

import micropython
import bno055 as imu
from time import sleep
accel_range = int('00000000',2) ##accel = 2g
accel_range = int('00000001',2) ##accel = 4g care if not in fusion mode. top 8 bits impact power and BW


if __name__ == "__main__":
    '''Reads or Writes to i2c peripheral. This loop reads and prints 10 euler vectors'''
    aye = pyb.I2C (1, pyb.I2C.MASTER)
    nerd = imu.BNO055(aye, 0x28, accel_range)
    for x in range(10):
        x_data = nerd.get_eul()
        sleep(100.0/1000.0)


    def scan_list():
        aye = pyb.I2C (1, pyb.I2C.MASTER)
        adr_list = []
        adr_list = pyb.I2C.scan(aye)
        return adr_list
