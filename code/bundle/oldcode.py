# Motor Tester
# with Cedar Grove NAU7802 FeatherWing
# 2020-12-11 v00 Cedar Grove Studios

import board
import time
from   pulseio            import PulseIn
from   digitalio          import DigitalInOut, Direction, Pull
from   simpleio           import map_range
from   cedargrove_nau7802 import NAU7802
from   adafruit_ina260    import INA260
from   cg_adafruit_motorkit  import MotorKit

rpm           = DigitalInOut(board.D4)
rpm.direction = Direction.INPUT
rpm.pull      = Pull.UP

DEFAULT_CHAN = 1   # Select load cell channel input; channel A=1, channel B=2
SAMPLE_AVG = 100   # Number of sample values to average
MAX_GR = 50       # Maximum (full-scale) display range in grams
MIN_GR = ((MAX_GR // 5 ) * -1)  # Calculated minimum display value
DEFAULT_GAIN = 64  # Default gain for internal PGA

# Load cell dime-weight calibration ratio; 2.268 oz / ADC_raw_measurement
# Obtained emperically; individual load cell dependent
CALIB_RATIO = 2.268 / 2445  # load cell serial# 4540-01 for gain x64

# Instantiate 24-bit load sensor ADC
print('*** Instantiate NAU7802 24-bit ADC wing')
nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=2)

# Instantiate INA260 power sensors wing
print('*** Instantiate INA260 power sensors wing')
ina260_0 = INA260(board.I2C(), address=0x40)
ina260_1 = INA260(board.I2C(), address=0x41)

# Instantiate both motor controller wings
print('*** Instantiate PCA9685 motor control wings')
motor_wing_0 = MotorKit(i2c=board.I2C(), address=0x60)
motor_wing_0.frequency = 360
print('psa frequency:', motor_wing_0.frequency)

throttle = .9

motor_wing_0.motor1.throttle = throttle

"""motor_wing_1 = MotorKit(i2c=board.I2C(), address=0x61)
print('motor_wing_1:', help(motor_wing_1))
time.sleep(10)"""

def zero_channel():
    # Initiate internal calibration for current channel; return raw zero offset value
    # Use when scale is started, a new channel is selected, or to adjust for measurement drift
    # Remove weight and tare from load cell before executing
    print('channel %1d calibrate.INTERNAL: %5s'
          % (nau7802.channel, nau7802.calibrate('INTERNAL')))
    print('channel %1d calibrate.OFFSET:   %5s'
          % (nau7802.channel, nau7802.calibrate('OFFSET')))
    zero_offset = read(100)  # Read average of 100 samples to establish zero offset
    print('...channel zeroed')
    return zero_offset

def select_channel(channel=1):
    # Selects a channel for reading.
    nau7802.channel = channel
    print('channel %1d selected' % (nau7802.channel))
    return

def get_tare(value=None):
    # Measure and store tare weight; return raw, grams, and ounces values
    if value is None:
        # Read average of 100 samples and store raw tare offset
        tare_offset = read(100)
        tare_state = True
    else:
        # Set raw tare offset to zero and disable tare display
        tare_offset = 0
        tare_state  = False
    tare_gr_offset = round(tare_offset * CALIB_RATIO, 3)
    tare_oz_offset = round(tare_gr_offset * 0.03527, 4)
    return tare_offset, tare_gr_offset, tare_oz_offset

def read(samples=100):
    # Read and average consecutive raw sample values; return average raw value
    sum = 0
    for i in range(0, samples):
        if nau7802.available:
            sum = sum + nau7802.read()
    return int(sum / samples)

def get_rpm():
    # Read a few pulses from 20-slot chopper wheel
    t0 = time.monotonic()
    tx = t0 + 0.05  # timeout after one second
    chopper_count = 0
    while chopper_count <= 5 and time.monotonic() <= tx:
        while rpm.value and time.monotonic() <= tx:
            pass
        while not rpm.value and time.monotonic() <= tx:
            pass
        chopper_count += 1
    period = time.monotonic() - t0
    if time.monotonic() > tx:
        return 0
    rev_per_min = 60 / (period * (20 / 5))
    return rev_per_min

# Instantiate and calibrate load cell inputs
print('*** Instantiate and calibrate load cells')
#clue.pixel[0] = (16, 16, 0)  # Set status indicator to yellow
print('    enable NAU7802 digital and analog power: %5s' % (nau7802.enable(True)))

nau7802.gain = DEFAULT_GAIN  # Use default gain
for i in range(1, 3):  # Zero and calibrate both channels
    nau7802.channel = i
    zero = zero_channel()

nau7802.channel = DEFAULT_CHAN        # Set to default channel
zero = zero_channel()                 # Re-calibrate and get raw zero offset value
tare, tare_gr, tare_oz = get_tare(0)  # Disable tare subtraction and display

### Main loop: Read sample, move bubble, and display values
#     Monitor Zeroing and Tare buttons
while True:
    value   = read(SAMPLE_AVG)
    mass_gr = round((value - zero - tare) * CALIB_RATIO, 3)
    mass_oz = round(mass_gr * 0.03527, 4)

    voltage = ina260_0.voltage
    current = ina260_0.current/100
    rpms    = get_rpm() / 100

    print('(%+6.3f, %+2.3f, %+1.3f, %+6.3f)' % (mass_gr, voltage * throttle, current, rpms))
    # print('raw value:', value, hex(value))