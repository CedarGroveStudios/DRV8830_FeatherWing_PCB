# Motor Tester
# with Cedar Grove NAU7802 FeatherWing
# and Cedar Grove DRV8830 I2C/Analog FeatherWing
# motor_tester_code.py
# 2022-02-08 v0308 Cedar Grove Studios

import board
import time
import countio
#from   pulseio        import PulseIn
#import pwmio
import digitalio
from   analogio       import AnalogIn
import neopixel
from   simpleio       import map_range


led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, auto_write=True)
pixel[0] = (( 5, 0, 5))  # Startup indicator: purple

# ========== Test Process Parameters ==========

THROTTLE_SWEEP = True

MOTOR_SETTLING_TIME = 0.250   # seconds

MOTOR_CONTROLLER      = 'DRV8830_ANALOG'   # Choose only one from 'DRV8830_DIGITAL', 'DRV8830_ANALOG'
#MOTOR_CONTROLLER_MODE = 'SLOW_DECAY'  # Choose only one from 'FAST_DECAY', 'SLOW_DECAY'

TORQUE_SENSOR = False   # Activate torque measurement components
TORQUE_SAMPLE_AVG = 50  # Number of sample values to average

POWER_SENSOR  = True   # Activate power sensor components
POWER_SUPPLY  = 5.0    # Power supply voltage (fixed value)

RPM_SENSOR    = True   # Activate RPM measurement components
RPM_SENSOR_SLICES = 'n/a'  # Number of RPM sensor counts per rotation

# ====================

PRINT_FORMAT = '(%+4.0f, %+4.0f, %+6.0f, %+7.2f, %+5.0f, %7.1f, %7.1f, %+6.0f, %+7.2f, %+5.0f, %7.1f)'
# torque_a, torque_b, throttle_0*100, volt_0, mamp_0, mwatt_0, rpms, throttle_1*100, volt_1, mamp_1, mwatt_1

# Motor Controller Interface Parameters
if MOTOR_CONTROLLER == 'DRV8830_ANALOG':
    import cedargrove_drv8830
    motor0 = cedargrove_drv8830.DRV8830(board.I2C(), address=0x60)
    motor1 = cedargrove_drv8830.DRV8830(board.I2C(), address=0x61)
    print(f' ** DRV8830_ANALOG PWM selected')

else:
    print('no motor controller interface selected')

motor0.throttle = 0  # stop the motors
motor1.throttle = 0
motor0.throttle = None  # coast the motors
motor1.throttle = None

if RPM_SENSOR:
    print('*** Instantiate RPM measurement system')
    rpm_a = countio.Counter(board.RX)

if TORQUE_SENSOR:
    print('*** Instantiate Torque Measurement Components')
    from cedargrove_nau7802 import NAU7802
    NAU7802_DEFAULT_GAIN = 128  # Default gain for internal PGA

    # Load cell dime-weight calibration ratio; 2.268 gm / ADC_raw_measurement
    # Obtained emperically; individual load cell dependent
    NAU7802_CALIB_RATIO_01 = 100 / 215235  # 100g at gain x128 for load cell serial#4540-01
    NAU7802_CALIB_RATIO_02 = 100 / 215240  # 100g at gain x128 for load cell serial#4540-02

    # Instantiate 24-bit load sensor ADC
    print(' ** Instantiate NAU7802 24-bit ADC wing')
    nau7802 = NAU7802(board.I2C(), address=0x2A, active_channels=2)

    # Instantiate and calibrate load cell inputs
    print(' ** Calibrate load cells')
    pixel[0] = (( 5, 5, 0))  # Calibrate indicator: yellow
    print('  ** Enable NAU7802 digital and analog power: %5s' % (nau7802.enable(True)))

if POWER_SENSOR:
    # Instantiate power sensors
    print('*** Instantiate power sensors')
    current_mon_0 = AnalogIn(board.A4)
    current_mon_1 = AnalogIn(board.A5)
    CURRENT_MON_SAMPLE_AVG = 5

def zero_channel():
    # Initiate internal calibration for current channel; return raw zero offset value
    # Use when scale is started, a new channel is selected, or to adjust for measurement drift
    # Remove weight and tare from load cell before executing
    print('channel %1d calibrate.INTERNAL: %5s'
          % (nau7802.channel, nau7802.calibrate('INTERNAL')))
    print('channel %1d calibrate.OFFSET:   %5s'
          % (nau7802.channel, nau7802.calibrate('OFFSET')))
    zero_offset = read_load_cell(100)  # Read average of 100 samples to establish zero offset
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

def read_load_cell(samples=100):
    # Read and average consecutive raw sample values; return average raw value
    sum = 0
    for i in range(0, samples):
        if nau7802.available:
            sum = sum + nau7802.read()
    return int(sum / samples)

def read_rpm(revolutions=1, n=1, min_rpm=30):
    # Read a few revolutions from n-slot chopper wheel using rpm_a sensor,
    #   measure the interval, calc RPM
    # min_rpm is used to set timeout value

    timeout = (60 / min_rpm) * n * 1.1  # set timeout to 110% of minimum rotational period

    led.value = False
    rpm_a.reset()

    t0 = time.monotonic()
    tx = t0 + timeout  # target timeout
    while (rpm_a.count <= revolutions * n) and (time.monotonic() <= tx):
        led.value = True
        rpm_a_end = rpm_a.count
        period = time.monotonic() - t0
    led.value = False

    if rpm_a_end == 0:
        #print('read_rpm: stall detected')
        return 0  # 0 RPM
    return (60 / (period / (rpm_a_end / n)))  # Calculate and return RPM

def read_volts_amps(samples=1):
    # Read voltage and current values of both sensors
    if motor0.throttle:
        v_0 = motor0.throttle_volts  # volts
        a_0 = map_range(current_mon_0.value, 0, 65535, 0, 1650)  # milliamps
        p_0 = abs(v_0 * a_0)  # milliwatts
    else:
        v_0 = a_0 = p_0 = 0

    if motor1.throttle:
        v_1 = motor1.throttle_volts
        a_1 = map_range(current_mon_1.value, 0, 65535, 0, 1650)  # milliamps
        p_1 = abs(v_1 * a_1)  # milliwatts
    else:
        v_1 = a_1 = p_1 = 0

    return v_0, a_0, p_0, v_1, a_1, p_1

def print_header():
    print(' torque      test motor                       brake motor')
    print(' ----------  -------------------------------  ----------------------------------------')
    print(' lc_a, lc_b, thrott,   volts,  mamp,   mwatt,     rpm, thrott,   volts,  mamp,   mwatt')
    print('(+---, +---, +-----, +---.--, +----, +----.-, +----.-, +-----, +---.--, +----, +----.-)')
    return

def print_params(title=''):
    print()
    print(f'  =============== {title} ===============')
    print(f'  Motor Controller: {MOTOR_CONTROLLER}')
    print(f'  Motor Settling Time : {MOTOR_SETTLING_TIME} sec')
    print(f'  Speed Sensor Active : {RPM_SENSOR}   Slices: {RPM_SENSOR_SLICES}')
    print(f'  Torque Sensor Active: {TORQUE_SENSOR}  Sample Average: {TORQUE_SAMPLE_AVG}')
    print()
    return

### Main loop:
while True:
    pixel[0] = (( 0, 5, 0))  # Running indicator: green
    if TORQUE_SENSOR:
        for i in range(1, 3):  # Zero and calibrate both channels
            nau7802.channel = i
            zero = zero_channel()

    motor0.throttle = 0
    motor1.throttle = 0   # Brake motor bias

    if THROTTLE_SWEEP:
        print_params('THROTTLE_SWEEP')
        print_header()
        for v in range(0, 101, 5):  # throttle scan
            pixel[0] = (( 0, 0, 5))  # Motor setup indicator: blue

            throttle = v/100  # variable speed
            motor0.throttle = throttle
            time.sleep(MOTOR_SETTLING_TIME)  # wait for motor speed to settle
            pixel[0] = (( 0, 5, 0))  # Running indicator: green

            if TORQUE_SENSOR:
                nau_7802_channel = 1  # load_cell_a
                load_cell_a = read_load_cell(TORQUE_SAMPLE_AVG)
                torque_a = round((load_cell_a) * NAU7802_CALIB_RATIO_01, 3) * 1000

                nau_7802_channel = 2  # load_cell_b
                load_cell_b = read_load_cell(TORQUE_SAMPLE_AVG)
                torque_b = round((load_cell_b) * NAU7802_CALIB_RATIO_02, 3) * 1000
            else:
                torque_a = 0
                torque_b = 0

            if POWER_SENSOR:
                volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(CURRENT_MON_SAMPLE_AVG)
            else:
                volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

            if RPM_SENSOR:
                rpms = read_rpm(revolutions=50, n=5, min_rpm=30)  # magnetic sensor on brake motor
            else:
                rpms = 0

            pixel[0] = (( 5, 5, 0))  # Measurement complete indicator: purple

            if motor0.throttle:
                throttle_0 = motor0.throttle * 100
            else:
                throttle_0 = 0

            if motor1.throttle:
                throttle_1 = motor1.throttle * 100
            else:
                throttle_1 = 0

            print(PRINT_FORMAT % (torque_a, torque_b, throttle_0, volt_0, mamp_0, mwatt_0, rpms, throttle_1, volt_1, mamp_1, mwatt_1))

        pixel[0] = (( 5, 0, 0))  # End of run indicator: red

        rpms = throttle = freq = 0
        print(PRINT_FORMAT % (torque_a, torque_b, throttle, volt_0, mamp_0, mwatt_0, rpms, throttle, volt_1, mamp_1, mwatt_1))

    motor0.throttle = 0
    motor1.throttle = 0


    while True:
        pass
