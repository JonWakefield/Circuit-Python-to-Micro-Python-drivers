''' Author: Jon Wakefield
    Date: 02/27/2023
    Company: USDA - ARS
    Description: Example program for the Garmin Lidar lite v3
                 Using Micropython with circuit python modules
                 
                 Modified example of:
                 https://github.com/adafruit/Adafruit_CircuitPython_LIDARLite/blob/main/examples/lidarlite_simpletest.py
                 
    Firmware:
            - pico-w-anvil-v0.1.2-firmware-only.uf2
            
    Hardware:
            - Raspberry Pi Pico H (RP2040)

'''

import time #standard micropython time library
import board #Modified CircuitPy board module
import busio #Modified CircuitPy busio module
import adafruit_lidarlite #Modified CircuitPy adafruit_lidarlite driver


# Initilize our i2c connection here:
# Syntax: (SCL connection, SDA connection, OPTIONAL: frequency (defaults to 100000))
# Can have multiple slaves running on 1 i2c port.
i2c = busio.I2C(board.GP5, board.GP4, frequency=100000)


# Pass our i2c object to LIDARLite class:
lidar_sensor = adafruit_lidarlite.LIDARLite(i2c)


# Forever loop to test i2c connection
while True:
    try:
        # Print our Lidar distance by calling the distance property
        print((lidar_sensor.distance))
    except RuntimeError as e:
        # Print any errors:
        print(e)
    # Can modify (increase or decrease) speed of lidar captures:
    time.sleep(0.1) 
    
    
    