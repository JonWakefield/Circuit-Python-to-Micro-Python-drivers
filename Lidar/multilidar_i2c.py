''' Author: Jon Wakfield

    Descriptin: I2C Support for Multiple Garmin Lidar Lite v3 connected to a master I2C device (Pi Pico)
                Using modified circuit python drivers to allow for micropython firmware support.
    
    Firmware: pico-w-anvil-v0.1.2 (may work on other MicroPython firmwares, untested).
    
    Related Documents & Documentation:
    https://github.com/JonWakefield/Circuit-Python-to-Micro-Python-drivers

'''


import board
import busio
import time
# from adafruit_bus_device.i2c_device import I2CDevice
from i2c_device import I2CDevice
import digitalio
from micropython import const
from machine import Pin

    
''' REGISTERS USED TO CHANGE I2C ADDRESS'''
_REG_I2C_ID_HIGH = const(0x18) # write device id high byte back to device
_REG_I2C_ID_LOW = const(0x19) # write device id low byte back to device
_REG_I2C_SEC_ADDR = const(0x1A) # write new address to this register:


_ADDR_DEFAULT = const(0x62)
_REG_ACQ_COMMAND = const(0x00)
_REG_DIST_MEAS_V3 = const(0x8F)
_REG_DIST_MEAS_V3HP = const(0x0F)
_REG_SIG_COUNT_VAL = const(0x02)
_REG_ACQ_CONFIG_REG = const(0x04)
_REG_THRESHOLD_BYPASS = const(0x1C)
_REG_STATUS = const(0x01)
_REG_UNIT_ID_HIGH = const(0x16)
_REG_UNIT_ID_LOW = const(0x17)
_REG_SIGNAL_STRENGTH = const(0x0E)
_REG_HEALTH_STATUS_V3HP = const(0x48)
_REG_POWER_CONTROL = const(0x65)
_REG_I2C_CONFIG = const(0x1E)
_REG_TEST_COMMAND = const(0x40)
_REG_CORR_DATA = const(0x52)

_CMD_RESET = const(0x00)
_CMD_DISTANCENOBIAS = const(0x03)
_CMD_DISTANCEWITHBIAS = const(0x04)
_CMD_DISTANCE_V3HP = const(0x03)
_NUM_DIST_BYTES = 2  # How many bytes is the returned distance measurement?

TYPE_V3 = "V3"
TYPE_V3HP = "V3HP"

CONFIG_DEFAULT = 0
CONFIG_SHORTFAST = 1
CONFIG_DEFAULTFAST = 2
CONFIG_MAXRANGE = 3
CONFIG_HIGHSENSITIVE = 4
CONFIG_LOWSENSITIVE = 5

"""Status Registers"""
# v3
STATUS_BUSY = 0x01
STATUS_REF_OVERFLOW = 0x02
STATUS_SIGNAL_OVERFLOW = 0x04
STATUS_NO_PEAK = 0x08
STATUS_SECOND_RETURN = 0x10
STATUS_HEALTHY = 0x20
STATUS_SYS_ERROR = 0x40

# v3 HP
STATUS_BUSY_V3HP = 0x01
STATUS_SIGNAL_OVERFLOW_V3HP = 0x02

# The various configuration register values, from arduino library
_LIDAR_CONFIGS = (
    (0x80, 0x08, 0x00),  # default
    (0x1D, 0x08, 0x00),  # short range, high speed
    (0x80, 0x00, 0x00),  # default range, higher speed short range
    (0xFF, 0x08, 0x00),  # maximum range
    (0x80, 0x08, 0x80),  # high sensitivity & error
    (0x80, 0x08, 0xB0),
)  # low sensitivity & error


class LIDARLite:
    """
    A driver for the Garmin LIDAR Lite laser distance sensor.
    Initialize the hardware for the LIDAR over I2C. You can pass in an
    optional reset_pin for when you call reset(). There are a few common
    configurations Garmin suggests: CONFIG_DEFAULT, CONFIG_SHORTFAST,
    CONFIG_DEFAULTFAST, CONFIG_MAXRANGE, CONFIG_HIGHSENSITIVE, and
    CONFIG_LOWSENSITIVE. For the I2C address, the default is 0x62 but if you
    pass a different number in, we'll try to change the address so multiple
    LIDARs can be connected. (Note all but one need to be in reset for this
    to work!)
    :param i2c_bus: The `busio.I2C` object to use. This is the only
                    required parameter.
    :param int address: (optional) The I2C address of the device to set
                        after initialization.
    """

    def __init__(
        self,
        i2c_bus,
        *,
        reset_pin=None,
        configuration=CONFIG_DEFAULT,
        address=_ADDR_DEFAULT,
        sensor_type=TYPE_V3,
    ):
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._buf = bytearray(2)
        self._bias_count = 0
        self._reset = reset_pin
        time.sleep(0.5)
        self.configure(configuration)
        self._status = self.status
        self._sensor_type = sensor_type

    def reset(self):
        """Hardware reset (if pin passed into init) or software reset. Will take
        100 readings in order to 'flush' measurement unit, otherwise data is off."""
        # Optional hardware reset pin
        if self._reset is not None:
            self._reset.direction = digitalio.Direction.OUTPUT
            self._reset.value = True
            self._reset.value = False
            time.sleep(0.01)
            self._reset.value = True
        else:
            try:
                self._write_reg(_REG_ACQ_COMMAND, _CMD_RESET)
            except OSError:
                print("OSError")
        time.sleep(1)
        # take 100 readings to 'flush' out sensor!
        for _ in range(100):
            try:
                self.read_distance_v3(True)
            except RuntimeError:
                print("RuntimeError")

    def configure(self, config):
        """Set the LIDAR desired style of measurement. There are a few common
        configurations Garmin suggests: CONFIG_DEFAULT, CONFIG_SHORTFAST,
        CONFIG_DEFAULTFAST, CONFIG_MAXRANGE, CONFIG_HIGHSENSITIVE, and
        CONFIG_LOWSENSITIVE."""
        settings = _LIDAR_CONFIGS[config]
        self._write_reg(_REG_SIG_COUNT_VAL, settings[0])
        self._write_reg(_REG_ACQ_CONFIG_REG, settings[1])
        self._write_reg(_REG_THRESHOLD_BYPASS, settings[2])

    def read_distance_v3(self, bias=False):
        """Perform a distance reading with or without 'bias'. It's recommended
        to take a bias measurement every 100 non-bias readings (they're slower)"""
        if bias:
            self._write_reg(_REG_ACQ_COMMAND, _CMD_DISTANCEWITHBIAS)
        else:
            self._write_reg(_REG_ACQ_COMMAND, _CMD_DISTANCENOBIAS)
        dist = self._read_reg(_REG_DIST_MEAS_V3, _NUM_DIST_BYTES)
        if self._status & (STATUS_NO_PEAK | STATUS_SECOND_RETURN):
            if self._status & STATUS_NO_PEAK:
                raise RuntimeError("Measurement failure STATUS_NO_PEAK")
            if self._status & STATUS_SECOND_RETURN:
                raise RuntimeError("Measurement failure STATUS_NO_PEAK")
            raise RuntimeError("Some other runtime error")

        if (self._status & STATUS_SYS_ERROR) or (not self._status & STATUS_HEALTHY):
            raise RuntimeError("System failure")
        return dist[0] << 8 | dist[1]

    def read_distance_v3hp(self):
        """Perform a distance measurement for the v3 HP sensor"""
        # Any non-zero value written to _REG_ACQ_COMMAND will start a reading on v3HP, no bias vs.
        #   non-bias
        self._write_reg(_REG_ACQ_COMMAND, _CMD_DISTANCEWITHBIAS)
        dist = self._read_reg(_REG_DIST_MEAS_V3HP, _NUM_DIST_BYTES)
        return dist[0] << 8 | dist[1]

    @property
    def correlation_data(self):
        """Reads correlation data"""
        # TODO: How to translate correlation data property?
        corr_data = self._read_reg(_REG_CORR_DATA, 2)
        return corr_data[0] << 8 | corr_data[1]

    @property
    def test_command(self):
        """Reads the test command"""
        return self._read_reg(_REG_TEST_COMMAND, 1)[0]

    @property
    def i2c_config(self):
        """Reads the I2C config"""
        return self._read_reg(_REG_I2C_CONFIG, 1)[0]

    @property
    def power_control(self):
        """Reads the power control register"""
        return self._read_reg(_REG_POWER_CONTROL, 1)[0]

    @property
    def health_status(self):
        """Reads health status for v3HP (not available on v3, will return -1)"""
        if self._sensor_type == TYPE_V3HP:
            return self._read_reg(_REG_HEALTH_STATUS_V3HP, 1)[0]

        return -1

    @property
    def signal_strength(self):
        """Reads the signal strength of the last measurement"""
        return self._read_reg(_REG_SIGNAL_STRENGTH, 1)[0]

    def unit_id(self, reg):
        """Reads the serial number of the unit"""
        byte = self._read_reg(reg, 1)
        print(hex(byte[0]))
        
        return byte[0]


    @property
    def distance(self):  # pylint: disable=R1710
        """The measured distance in cm. Will take a bias reading every 100 calls"""
        self._bias_count -= 1

        if self._bias_count < 0:
            self._bias_count = 100  # every 100 reads, check bias
        if self._sensor_type == TYPE_V3:
            return self.read_distance_v3(self._bias_count <= 0)
        if self._sensor_type == TYPE_V3HP:
            return self.read_distance_v3hp()

        # If no sensor type has been identified, return a negative distance as an error
        return -1.0

    @property
    def status(self):
        """The status byte, check datasheet for bitmask"""
        buf = bytearray([_REG_STATUS])
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buf, buf)
        return buf[0]

    def _write_reg(self, reg, value):
        self._buf[0] = reg # store the register we want to write to in the first position of the bytearray
        self._buf[1] = value # store the value we wish to write in the second position of the bytearray
        with self.i2c_device as i2c:
        #    print("Writing: ", [hex(i) for i in self._buf])
            i2c.write(self._buf)
        time.sleep(0.001)  # there's a delay in arduino library

    def _read_reg(self, reg, num):
        while True:
            self._status = self.status
            if not self._status & STATUS_BUSY:
                break
        # no longer busy
        self._buf[0] = reg # store register in first location of the buffer
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buf, self._buf, out_end=1, in_end=num)
        #print("Read from ", hex(reg), [hex(i) for i in self._buf])
        return self._buf
    
    def change_i2c_address(self, new_address: int, i2c_bus):
        ''' Change i2c address for each Lidar unit'''
        
        
        disable_default_i2c_address = const(0x08) # 4th step. write to _REG_I2C_CONFIG to disable default i2c address (0x62)
        
        ''' First Read in the device ID'''
        unq_id_high = self.unit_id(_REG_UNIT_ID_HIGH)
        unq_id_low = self.unit_id(_REG_UNIT_ID_LOW)
        print(f"unq_id_high is: {unq_id_high}")
        print(f"unq_id_low is: {unq_id_low}")
        
 #       unq_id_high = 64
        
        ''' 2nd) Write device id back to device'''
        self._write_reg(_REG_I2C_ID_HIGH, unq_id_high) # write high byte to 0x18
        self._write_reg(_REG_I2C_ID_LOW, unq_id_low) # write low byte to 0x19
        
        ''' 3rd) Write the new desired address to 0x1a '''
        self._write_reg(_REG_I2C_SEC_ADDR, new_address)
        
        '''' 4th) Write 0x08 to 0x1e to disable default I2C address '''
        self._write_reg(_REG_I2C_CONFIG, disable_default_i2c_address)

        
        # Finally, we need to re-initlize the i2c_device with the new address
        self.i2c_device = I2CDevice(i2c_bus, new_address)
        
        
        
# class FIRFilter:
#     def __init__(self, taps, queue_size=100):
#         self.taps = taps
#         self.queue_size = queue_size
#         self.queue = ucollections.deque(maxlen=queue_size)
#         
#     def filter(self, data):
#         self.queue.extend(data)
#         output = 0
#         for i, tap in enumerate(self.taps):
#             output += tap * self.queue[-i-1]
#         return output
    
        


def i2c_scanner() -> int:
    '''Scan for I2C address on I2C bus0
        Ensure all 4 lidars are connected
        If 4 devices are found function returns 4
        else Send error message to RPi ? '''
    
    x = 0
    devices_found = 0
    while not i2c_1.try_lock():
        pass

    try:
        while x < 1:
            print(
                "I2C addresses found:",
                [hex(device_address) for device_address in i2c_1.scan()],
            )
            for device_address in i2c_1.scan():
                devices_found += 1 
            time.sleep(1)
            x += 1

    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c_1.unlock()
        
    print(f"Found {devices_found} devices")

    return devices_found


def setup_lidars(lidar1_PE, lidar2_PE, lidar3_PE, lidar4_PE):
    '''Setup all 4 Lidar devices:
        How setup Works:
        1. Change I2C addresses 1 by 1:
            1a) Drive all four lidar power enables low
            1b) Turn lidar 1s pe to high, configure lidar unit and change address
            1a) Turn lidar 2s pe to high, configure ldiar unit and change address
            1b) Repeat for devices 3 & 4
    '''
    
    # First: Drive all 4 power enables low
    lidar1_PE.value(0)
    lidar2_PE.value(0)
    lidar3_PE.value(0)
    lidar4_PE.value(0)
    time.sleep(2)
    
    # Second: set Power Enable high & create lidar1 obj & change address
    print("Setting up lidar 1: ")
    try:
        lidar1_PE.value(1)
        time.sleep(2) # sleep for a second to ensure lidar is back to powered on state
        lidar_1 = LIDARLite(i2c_1)
        lidar_1.change_i2c_address(lidar_1_i2c_address, i2c_1)
    except (ValueError) as e:
        errMsg = "Could not set-up Lidar 1"
        print(e)
        lidar_1 = -99
        print(errMsg)
    
    
    # Third: Set lidar2 PE high, create lidar2 instance & change default address
    print("Setting up lidar 2:")
    try:
        time.sleep(1)
        lidar2_PE.value(1)
        time.sleep(1)
        lidar_2 = LIDARLite(i2c_1)
        lidar_2.change_i2c_address(lidar_2_i2c_address, i2c_1)
    except (ValueError) as e:
        errMsg = "Could not set-up Lidar 2"
        lidar_2 = -99
        print(errMsg)
    

    # Fourth: Repeat same steps for lidars3 & lidars4
    print("Setting up lidar 3:")
    try:
        time.sleep(1)
        lidar3_PE.value(1)
        time.sleep(1)
        lidar_3 = LIDARLite(i2c_1)
        lidar_3.change_i2c_address(lidar_3_i2c_address, i2c_1)
    except (ValueError) as e:
        errMsg = "Could not set-up Lidar 3"
        lidar_3 = -99
        print(errMsg)
    
    
    # Activate 4th Lidar & change address:
    print("Setting up lidar 4:")
    try:
        time.sleep(1)
        lidar4_PE.value(1)
        time.sleep(1)
        lidar_4 = LIDARLite(i2c_1)
        lidar_4.change_i2c_address(lidar_4_i2c_address, i2c_1)
    except (ValueError) as e:
        errMsg = "Could not set-up Lidar 4"
        lidar_4 = -99 
        print(errMsg)
    
    
    return lidar_1, lidar_2, lidar_3, lidar_4



def read_lidars(lidar1, lidar2, lidar3, lidar4, dist_bias) -> int:
    ''' Function Description:
        Read in distance & RSSI data for reach lidar unit:
        [Function is based upon read_lidars from lidarlite_test_r8.py]
    '''
    
    dist_lidar1 = -9999
    dist_lidar2 = -9999 
    dist_lidar3 = -9999
    dist_lidar4 = -9999
    rssi_lidar1 = -9999
    rssi_lidar2 = -9999
    rssi_lidar3 = -9999
    rssi_lidar4 = -9999
    
    # read in data (distance + rssi) from lidar 1:
    try:
        dist_lidar1 = int(lidar1.distance) + dist_bias[0]
        rssi_lidar1 = lidar1.signal_strength
        print(f"Lidar 1 Distance: {dist_lidar1}")
        #print(f"Lidar 1 RSSI: {rssi_lidar1}")
        
    except:
        err_msg = "Error reading in data from lidar 1!"
        # Send -99 back to Rpi informing Main system a lidar devices is not connected!
        dist_lidar1 = -99
        rssi_lidar1 = -99
        print(err_msg)
        
        
    # Read in data from Lidar #2:
    try:
        dist_lidar2 = int(lidar2.distance) + dist_bias[1]
        rssi_lidar2 = lidar2.signal_strength
        print(f"Lidar 2 Distance: {dist_lidar2}")
        #print(f"Lidar 2 RSSI: {rssi_lidar2}")
        
    except:
        err_msg = "Error reading in data from lidar 2!"
        dist_lidar2 = -99
        rssi_lidar2 = -99
        print(err_msg)
        
        
    # Read in data from Lidar #3:    
    try:
        dist_lidar3 = int(lidar3.distance) + dist_bias[2]
        rssi_lidar3 = lidar3.signal_strength
        print(f"Lidar 3 Distance: {dist_lidar3}")
        #print(f"Lidar 3 RSSI: {rssi_lidar3}")
        
    except:
        err_msg = "Error reading in data from lidar 3!"
        dist_lidar3 = -99
        rssi_lidar3 = -99
        print(err_msg)
        
        
    # Read in data from Lidar #4:
    try:
        dist_lidar4 = int(lidar4.distance) + dist_bias[3]
        rssi_lidar4 = lidar4.signal_strength
        print(f"Lidar 4 Distance: {dist_lidar4}")
        #print(f"Lidar 4 RSSI: {rssi_lidar4}")
        
    except:
        err_msg = "Error reading in data from lidar 4!"
        dist_lidar4 = -99
        rssi_lidar4 = -99
        print(err_msg)
        
    # ...
    return [dist_lidar1, dist_lidar2, dist_lidar3, dist_lidar4], [rssi_lidar1, rssi_lidar2, rssi_lidar3, rssi_lidar4]



def low_pass_filter(distance: int, lpf_distance: float ) -> float:
    
    lpf_distance = (lpf_distance + distance) / 2.0
    
    return lpf_distance



def check_if_beam_broken(lidar_distances: List, lpf_distances: list) -> int:
    ''' Loop Through Distances from each lidar,
        checking if we detected a beam break.
    '''
    
    # ------- Local Variables -------------
    short_dist_threshold = 220 #243cm == 8ft  so back off 12" == 30.5cm
    long_dist_threshold = 285
    breaks_detected_lidar1 = 0
    breaks_detected_lidar2 = 0
    breaks_detected_lidar3 = 0
    breaks_detected_lidar4 = 0
    # -------------------------------------
    
    
    
    for i in range(len(lidar_distances)):
        
        distance = lidar_distances[i]
        
        if (distance > 0):
            
            lpf_distances[i] = low_pass_filter(distance, lpf_distances[i])
            #print(f"Low Pass Filter Distance: {lpf_distances[i]}")
            
            if((distance < short_dist_threshold) or (distance > long_dist_threshold)):
                if( i == 0):
                    breaks_detected_lidar1 += 1
                elif (i == 1):
                    breaks_detected_lidar2 += 1
                elif ( i == 2):
                    breaks_detected_lidar3 += 1
                elif (i == 3):
                    breaks_detected_lidar4 += 1
                
                #print(f"Break Detected from Lidar {i+1}")
                
                # ...[more to do here]
        #else:
            #print(f"Distance < 0, distance value: {distance}")
            
    
    
    return breaks_detected_lidar1, breaks_detected_lidar2, breaks_detected_lidar3, breaks_detected_lidar4, lpf_distances


def update_lidar_bias(lidar1_distances: list, lidar2_distances: list, lidar3_distances: list, lidar4_distances: list, active_lidars: int, dist_bias: list ) -> int:
    ''' Function Description:
        After every 100 iterations, call function to perform a distance biasing update
        Helps ensure Lidars are reading in correct values:
        
        # NOTE: MAY NEED A UNQ BIAS FOR EACH LIDAR
    '''
    
    median = int(len(lidar1_distances) / 2)
    
    # First: sort lidar distance Lists:
    lidar1_distances.sort()
    lidar2_distances.sort()
    lidar3_distances.sort()
    lidar4_distances.sort()
    
    # SECOND: Take median distance from each lidar list
    lidar1_median = lidar1_distances[median]
    lidar2_median = lidar2_distances[median]
    lidar3_median = lidar3_distances[median]
    lidar4_median = lidar4_distances[median]
    
    lidar_medians = [lidar1_median, lidar2_median, lidar3_median, lidar4_median]
    
    print(f"Lidar 1 median: {lidar1_median}")
    print(f"Lidar 2 median: {lidar2_median}")
    print(f"Lidar 3 median: {lidar3_median}")
    print(f"Lidar 4 median: {lidar4_median}")
    
    # THIRD: Average the median value from each Lidar:
    avg_median = (lidar1_median + lidar2_median + lidar3_median + lidar4_median) / active_lidars
    
    print(f"Average median: {avg_median}")
    
    # FOURTH: Determine if we need to +- the distance bias for each indivudal lidar
    for i in range(active_lidars - 1):
        
        difference = avg_median - lidar_medians[i]
        dist_bias[i] = difference

        
    
    return dist_bias



''' LIDAR I2C ADDRESSES:

    LIDAR_1 : 0x64 (d'100)
    LIDAR_2 : 0x66 (d'102)
    LIDAR_3 : 0x68 (d'104)
    LIDAR_4 : 0x6A (d'106) 

'''
lidar_1_i2c_address = 100 #0x64
lidar_2_i2c_address = 102 #0x66 
lidar_3_i2c_address = 104 #0x68
lidar_4_i2c_address = 106 #0x6A



''' Configure i2c connection using busio to GP pins on Pi Pico'''
i2c_1 = busio.I2C(board.GP5, board.GP4) # 


'''Power Enable pins: '''
# Setup digital IO pins
# lidar1_PE = digitalio.DigitalInOut(board.GP0)
# lidar2_PE = digitalio.DigitalInOut(board.GP5)
# lidar3_PE = digitalio.DigitalInOut(board.GP6)
# lidar4_PE = digitalio.DigitalInOut(board.GP7)

lidar1_PE = Pin(15, Pin.OUT)
lidar2_PE = Pin(14, Pin.OUT)
lidar3_PE = Pin(13, Pin.OUT)
lidar4_PE = Pin(12, Pin.OUT)

# configure IO pins for output
# lidar1_PE.direction = digitalio.Direction.OUTPUT
# lidar2_PE.direction = digitalio.Direction.OUTPUT
# lidar3_PE.direction = digitalio.Direction.OUTPUT
# lidar4_PE.direction = digitalio.Direction.OUTPUT


# --------- Global Variables: -------------------

total_breaks_detected_lidar1 = 0 # breaks detected from lidar 1
total_breaks_detected_lidar2 = 0
total_breaks_detected_lidar3 = 0
total_breaks_detected_lidar4 = 0
all_breaks_detected = 0 # sum of all breaks detected from all lidar devices

total_iterations = 10 # number of iterations befor performing a distance biasing check


# List used to store LPF distances for each lidar( lpf_distances[0] -> lidar 1, lpf_distances[1] -> lidar 2 ... etc.)
# INTIAL LIDAR DISTANCE BIAS
lpf_distances = [270.0 , 270.0 , 270.0 , 270.0 ] #

# lists used to store distances values wrt each lidar: (used for bias correction)
lidar1_distances_list = [-99 for i in range(total_iterations)]
lidar2_distances_list = [-99 for i in range(total_iterations)]
lidar3_distances_list = [-99 for i in range(total_iterations)]
lidar4_distances_list = [-99 for i in range(total_iterations)]

# distance biasing for Lidar units
# NOTE: MAY NEED A UNQ BIAS FOR EACH LIDAR
dist_bias = [0,0,0,0]

x = 0
# -----------------------------------------------


if __name__ == '__main__':
    
    # initlize the filter:
#     fir_filter = FIRFilter(lpf_distances)
    
    # FIRST: Setup lidars using adafruit LIDARLite class: 
    lidar1, lidar2, lidar3, lidar4 = setup_lidars(lidar1_PE, lidar2_PE, lidar3_PE, lidar4_PE)
    
    
    while (True):
        
        # SECOND: Do a i2c scan to ensure all devices are connected:
        lidars_connected = i2c_scanner()
        if(lidars_connected != 4):
            print(f"Only {lidars_connected} Lidar Devices Found!")
            
            # ... Send message to RPI informing main system ?
    
            
        # For now, just loop through n times: (will be updated)    
        for i in range(total_iterations):
        
        
            # THIRD: read in data (distance , RSSI) from each lidar
            lidar_distances, lidar_rssis = read_lidars(lidar1, lidar2, lidar3, lidar4, dist_bias)
            
            
            # pass data to FIR filter:
            
            
            # FOURTH: Store lidar distance values in list for bias updating:
            lidar1_distances_list[i] = lidar_distances[0]
            lidar2_distances_list[i] = lidar_distances[1]
            lidar3_distances_list[i] = lidar_distances[2]
            lidar4_distances_list[i] = lidar_distances[3]
            
                
                
            # FIFTH: Check if beam is broken
            breaks1, breaks2, breaks3, breaks4, lpf_distances = check_if_beam_broken(lidar_distances, lpf_distances)
                
            # Increment Total number of breaks detected for each lidar
            total_breaks_detected_lidar1 += breaks1
            total_breaks_detected_lidar2 += breaks2
            total_breaks_detected_lidar3 += breaks3
            total_breaks_detected_lidar4 += breaks4
            
            
                
            # All breaks detected for all Lidars:    
            all_breaks_detected = total_breaks_detected_lidar1 + total_breaks_detected_lidar2 + total_breaks_detected_lidar3 + total_breaks_detected_lidar4
            
            
            
            # Once we've detected a beam break, need to inform RPI...
            # ...
            # ...
            
            
        # SIXTH: # After Looping Through 100 times: performing distance biasing update on LiDARS:
        # NOTE: MAY NEED A UNQ BIAS FOR EACH LIDAR
        dist_bias = update_lidar_bias(lidar1_distances_list, lidar2_distances_list, lidar3_distances_list, lidar4_distances_list, int(lidars_connected), dist_bias)
        
        x += 1
        
        if (x == 1):
            break
        
        
        
    print(f"Lidar 1 Total Breaks: {total_breaks_detected_lidar1}")
    print(f"Lidar 2 Total Breaks: {total_breaks_detected_lidar2}")
    print(f"Lidar 3 Total Breaks: {total_breaks_detected_lidar3}")
    print(f"Lidar 4 Total Breaks: {total_breaks_detected_lidar4}")
        
    print(f"Final LPF for Lidar 1: {lpf_distances[0]}")
    print(f"Final LPF for Lidar 2: {lpf_distances[1]}")
    print(f"Final LPF for Lidar 3: {lpf_distances[2]}")
    print(f"Final LPF for Lidar 4: {lpf_distances[3]}")
    
    #print(f"Lidar 1 distance values: {lidar1_distances_list}")
    #print(f"Lidar 2 distance values: {lidar2_distances_list}")
    #print(f"Lidar 3 distance values: {lidar3_distances_list}")
    #print(f"Lidar 4 distance values: {lidar4_distances_list}")
       
    print(f"Total number of breaks Detected: {all_breaks_detected}")
        
        
        
        
        
        
        
        
        



''' SYSTEM OVERVIEW:

    1) Setup Lidars using adafruit LIDAR class & change I2C address   (setup_lidars())
    2) Do i2c scan to ensure all devices are configured properly      (i2c_scanner())
    3) read in data (distance , rssi)                                 (read_lidars())
    4) check for beam breaks on each lidar                            (check_if_beam_broken())
    5) increate total number of beam breaks   



    TODO:
    
        1) Sending values (distance, break detection) back to RPI
        2) Receiving Data from RPI (restart ?, kill switch ?)
        3) Distance biasing
        4) Trigger signal to switch a SSR ( when a break is detected)
        5) After n readings, re-check i2c_scanner() to ensure all devices are still connected
        6) Finalize Error Handeling (Allow RPI to call setup_lidars to re-initlize them if problem encountered)

'''


ÿ