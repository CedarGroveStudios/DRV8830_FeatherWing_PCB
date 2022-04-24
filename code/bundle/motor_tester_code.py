# Motor Tester
# with Cedar Grove NAU7802 FeatherWing
# motor_tester_code.py
# 2021-03-20 v03 Cedar Grove Studios

import board
import time
from   countio        import Counter
from   pulseio        import PulseIn
import pwmio
from   digitalio      import DigitalInOut, Direction, Pull
import neopixel
from   simpleio       import map_range
from   adafruit_motor import motor

import motorkit_pwm_frequencies as motorkit_params
import breakout_pwm_frequencies as breakout_params

led = DigitalInOut(board.D13)
led.direction = Direction.OUTPUT

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, auto_write=True)
pixel[0] = (( 5, 0, 5))  # Startup indicator: purple

# ========== Test Process Parameters ==========

THROTTLE_SWEEP = False
FIXED_PWM_FREQ = 27000

FREQUENCY_SWEEP = False
FIXED_THROTTLE = 0.190

THRESHOLD_SWEEP = True

MOTOR_SETTLING_TIME = 0.250   # seconds

MOTOR_CONTROLLER      = 'BREAKOUT'   # Choose only one from 'BREAKOUT', 'CRICKIT', 'MOTORWING'
MOTOR_CONTROLLER_MODE = 'SLOW_DECAY'  # Choose only one from 'FAST_DECAY', 'SLOW_DECAY'

TORQUE_SENSOR = False   # Activate torque measurement components
TORQUE_SAMPLE_AVG = 50  # Number of sample values to average

POWER_SENSOR  = False   # Activate dual INA260 power sensor components

RPM_SENSOR    = True   # Activate RPM measurement components
RPM_SENSOR_SLICES = 'n/a'  # Number of RPM sensor counts per rotation

# ====================

# Motor Controller Interface Parameters
if MOTOR_CONTROLLER_MODE == 'SLOW_DECAY':
    DECAY_MODE = motor.SLOW_DECAY
else:
    DECAY_MODE = motor.FAST_DECAY

PRINT_FORMAT = '(%9.4f, %+4.0f, %+4.0f, %+6.0f, %+7.1f, %+5.0f, %+7.1f, %+7.1f)'
# freq, torque_a, torque_b, throttle * 1000, volt_0 * throttle * 200, mamp_0, mwatt_0, rpms
PRINT_FORMAT_2 = '(%9.3f, %+4.0f, %+4.0f, %+4.3f, %+3.2f, %+4.3f, %+4.3f, %+7.3f)'

if MOTOR_CONTROLLER == 'MOTORWING':
    from adafruit_motorkit import MotorKit
    print('*** Instantiate MotorWing/Shield (PCA9685) motor controller; motor1, motor2')
    MOTORWING_DEFAULT_PWM_FREQ = 1600  # Hz
    MOTORWING_MIN_PWM_FREQ     =   25  # Hz
    MOTORWING_MAX_PWM_FREQ     = 2100  # Hz

    motorwing_0 = MotorKit(i2c=board.I2C(), address=0x60)  # for wing or shield
    motor1 = motorwing_0.motor1
    motor2 = motorwing_0.motor2

    motorwing_0.frequency = FIXED_PWM_FREQ  # Set MotorWing to fixed PWM frequency
    motor1.decay_mode = DECAY_MODE
    motor2.decay_mode = DECAY_MODE

    print(f' ** MotorWing (PCA9685) frequency: {motorwing_0.frequency}  Mode code: {motor1.decay_mode}')

elif MOTOR_CONTROLLER == 'CRICKIT':
    print('*** Instantiate Crickit motor controller; motor1, motor2')
    from adafruit_crickit import crickit
    CRICKIT_DEFAULT_PWM_FREQ =    50  # Hz
    CRICKIT_MIN_PWM_FREQ     =     1  # Hz
    CRICKIT_MAX_PWM_FREQ     = 100000  # Hz

    motor1 = crickit.dc_motor_1
    motor2 = crickit.dc_motor_2

    ss = crickit.seesaw  # To access Seesaw motor pins
    ss.set_pwm_freq(22, FIXED_PWM_FREQ)  # Set motor1A pin to fixed PWM frequency
    ss.set_pwm_freq(23, FIXED_PWM_FREQ)  # Set motor1B pin to fixed PWM frequency
    ss.set_pwm_freq(19, FIXED_PWM_FREQ)  # Set motor2A pin to fixed PWM frequency
    ss.set_pwm_freq(18, FIXED_PWM_FREQ)  # Set motor2B pin to fixed PWM frequency
    print(' ** Crickit PWM frequency:', FIXED_PWM_FREQ)

elif MOTOR_CONTROLLER == 'BREAKOUT':
    print('*** Instantiate breakout motor controller; motor1')
    from adafruit_motor import motor
    BREAKOUT_DEFAULT_PWM_FREQ =   500  # Hz
    BREAKOUT_MIN_PWM_FREQ     =     1  # Hz
    BREAKOUT_MAX_PWM_FREQ     = 50000  # Hz

    pwm_a = pwmio.PWMOut(board.D5, frequency = FIXED_PWM_FREQ, variable_frequency=True)  # Set to default PWM frequency
    pwm_b = pwmio.PWMOut(board.D6, frequency = FIXED_PWM_FREQ, variable_frequency=True)
    motor1 = motor.DCMotor(pwm_a, pwm_b)
    motor2 = motor.DCMotor(pwm_a, pwm_b)
    print(f' ** Breakout PWM frequency: {pwm_a.frequency}')

    motor1.decay_mode = DECAY_MODE
    motor2.decay_mode = DECAY_MODE

else:
    print('no motor controller interface selected')

motor1.throttle = 0  # stop the motors
motor2.throttle = 0

if RPM_SENSOR:
    print('*** Instantiate RPM measurement system')
    rpm_a = Counter(board.TX)
    rpm_b = Counter(board.RX)

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
    # Instantiate INA260 power sensors wing; continuous mode (default)
    print('*** Instantiate INA260 power sensors wing')
    from adafruit_ina260 import INA260
    INA260_SAMPLE_AVG = 5
    ina260_0 = INA260(board.I2C(), address=0x40)
    ina260_1 = INA260(board.I2C(), address=0x41)

def set_pwm_freq(freq):
    if 'MOTORWING' in MOTOR_CONTROLLER:
        motorwing_0.frequency = freq
        freq = motorwing_0.frequency
        f_max = MOTORWING_MAX_PWM_FREQ
        f_min = MOTORWING_MIN_PWM_FREQ
    elif 'CRICKIT' in MOTOR_CONTROLLER:
        ss.set_pwm_freq(22, freq)  # Set motor1A pin to PWM frequency
        ss.set_pwm_freq(23, freq)  # Set motor1B pin to PWM frequency
        ss.set_pwm_freq(19, freq)  # Set motor2A pin to PWM frequency
        ss.set_pwm_freq(18, freq)  # Set motor2B pin to PWM frequency
        f_max = CRICKIT_MAX_PWM_FREQ
        f_min = CRICKIT_MIN_PWM_FREQ
    elif 'BREAKOUT' in MOTOR_CONTROLLER:
        pwm_a.frequency = int(freq)  # Set to PWM frequency
        pwm_b.frequency = int(freq)
        f_max = BREAKOUT_MAX_PWM_FREQ
        f_min = BREAKOUT_MIN_PWM_FREQ
    else:
        print('no motor controller interface selected')
    return freq, f_max, f_min

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
    ina260_0.averaging_count = int(samples)
    ina260_1.averaging_count = int(samples)
    v_0 = ina260_0.voltage  # volts
    a_0 = ina260_0.current  # milliamps
    p_0 = ina260_0.power  # milliwatts
    v_1 = ina260_1.voltage
    a_1 = ina260_1.current
    p_1 = ina260_1.power
    return v_0, a_0, p_0, v_1, a_1, p_1

def print_header():
    print('      freq, lc_a, lc_b, thrott,   volts,  mamp,   mwatt,     rpm')
    print('(----.----, +---, +---, +-----, +----.-, +----, +----.-, +----.-)')
    return

def print_params(title=''):
    print()
    print(f'  =============== {title} ===============')
    print(f'  Motor Controller: {MOTOR_CONTROLLER}  Mode: {MOTOR_CONTROLLER_MODE}')
    print(f'  Motor Settling Time: {MOTOR_SETTLING_TIME} sec')
    print(f'  Speed Sensor Active: {RPM_SENSOR}  Slices: {RPM_SENSOR_SLICES}')
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

    motor1.throttle = 0
    motor2.throttle = 0

    if THROTTLE_SWEEP:
        print_params('THROTTLE_SWEEP')
        print_header()
        for v in range(0, 101, 2):  # throttle scan
            pixel[0] = (( 0, 0, 5))  # Motor setup indicator: blue

            freq, f_max, f_min = set_pwm_freq(FIXED_PWM_FREQ)  # fixed frequency
            throttle = v/100  # variable speed
            motor1.throttle = throttle
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
                volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(INA260_SAMPLE_AVG)
            else:
                volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

            if RPM_SENSOR:
                #rpms = read_rpm(revolutions=50, n=5, min_rpm=30)  # magnetic sensor on brake motor
                #rpms = read_rpm(revolutions=3, n=20, min_rpm=10)  # optical sensor chopper wheel on TT gearmotor
                #rpms = read_rpm(revolutions=10, n=13, min_rpm=30)  # magnetic sensor on 7v gearmotor-encoder
                rpms = read_rpm(revolutions=100, n=7, min_rpm=100)  # magnetic sensor on N20 gearmotor-encoder
            else:
                rpms = 0

            pixel[0] = (( 5, 5, 0))  # Measurement complete indicator: purple

            print(PRINT_FORMAT % (freq, torque_a, torque_b, throttle * 1000, volt_0 * 200, mamp_0, mwatt_0, rpms))

        pixel[0] = (( 5, 0, 0))  # End of run indicator: red

        rpms = throttle = freq = 0
        print(PRINT_FORMAT % (freq, torque_a, torque_b, throttle * 1000, volt_0 * 200, mamp_0, mwatt_0, rpms))

    motor1.throttle = 0
    motor2.throttle = 0

    if FREQUENCY_SWEEP:
        print_params('FREQUENCY_SWEEP')
        print_header()
        for i in range(0, 5):  # 5 octave frequency scan
            for j in range(1, 10,):
                freq, f_max, f_min = set_pwm_freq(j * (10 ** i))
                if freq <= f_max and freq >= f_min:
                    pixel[0] = (( 0, 0, 5))  # Motor setup indicator: blue

                    throttle = FIXED_THROTTLE  # fixed throttle for frequency sweep
                    motor1.throttle = throttle
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
                        volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(INA260_SAMPLE_AVG)
                    else:
                        volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

                    if RPM_SENSOR:
                        #rpms = read_rpm(revolutions=100, n=5, min_rpm=30)  # magnetic sensor on brake motor
                        #rpms = read_rpm(revolutions=3, n=20, min_rpm=10)  # optical sensor chopper wheel on TT gearmotor
                        #rpms = read_rpm(revolutions=10, n=13, min_rpm=30)  # magnetic sensor on 7v gearmotor-encoder
                        rpms = read_rpm(revolutions=100, n=7, min_rpm=100)  # magnetic sensor on N20 gearmotor-encoder
                    else:
                        rpms = 0

                    pixel[0] = (( 5, 5, 0))  # Measurement complete indicator: purple

                    print(PRINT_FORMAT % (freq / 1000, torque_a, torque_b, throttle * 1000, volt_0 * 200, mamp_0, mwatt_0, rpms))

        pixel[0] = (( 5, 0, 0))  # End of run indicator: red

        motor1.throttle = 0
        motor2.throttle = 0

        rpms = throttle = freq = 0

        if POWER_SENSOR:
            time.sleep(1)
            volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(INA260_SAMPLE_AVG)
        else:
            volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

        print(PRINT_FORMAT % (freq, torque_a, torque_b, throttle * 1000, volt_0 * 200, mamp_0, mwatt_0, rpms))

    if THRESHOLD_SWEEP:
        print_params('THRESHOLD_SWEEP')
        print_header()
        for i in range(0, 5):  # 5 octave frequency scan
            for j in range(1, 10,):
                freq, f_max, f_min = set_pwm_freq(j * (10 ** i))
                if freq <= f_max and freq >= f_min:
                    pixel[0] = (( 0, 0, 5))  # Motor setup indicator: blue

                    for v in range(100, -1, -2):  # start with full throttle to reduce measurement time

                        throttle = v / 100
                        motor1.throttle = throttle
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
                            volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(INA260_SAMPLE_AVG)
                        else:
                            volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

                        if RPM_SENSOR:
                            #rpms = read_rpm(revolutions=10, n=5, min_rpm=30)  # magnetic sensor on brake motor
                            #rpms = read_rpm(revolutions=3, n=20, min_rpm=10)  # optical sensor chopper wheel on TT gearmotor
                            #rpms = read_rpm(revolutions=10, n=13, min_rpm=30)  # magnetic sensor on 7v gearmotor-encoder
                            rpms = read_rpm(revolutions=10, n=7, min_rpm=100)  # magnetic sensor on N20 gearmotor-encoder
                        else:
                            rpms = 0

                        pixel[0] = (( 5, 5, 0))  # Measurement complete indicator: purple

                        if round(rpms,3) == 0 or throttle == 0:
                            print(PRINT_FORMAT_2 % (old_freq / 1000, old_torque_a, old_torque_b, old_throttle, old_volt_0, old_mamp_0/1000, old_mwatt_0/1000, old_rpms))
                            break

                        old_freq = freq
                        old_torque_a = torque_a
                        old_torque_b = torque_b
                        old_throttle = throttle
                        old_volt_0 = volt_0
                        old_mamp_0 = mamp_0
                        old_mwatt_0 = mwatt_0
                        old_rpms = rpms

        pixel[0] = (( 5, 0, 0))  # End of run indicator: red

        motor1.throttle = 0
        motor2.throttle = 0

        rpms = throttle = freq = 0

        if POWER_SENSOR:
            time.sleep(1)
            volt_0, mamp_0, mwatt_0, volt_1, mamp_1, mwatt_1 = read_volts_amps(INA260_SAMPLE_AVG)
        else:
            volt_0 = mamp_0 = mwatt_0 = volt_1 = mamp_1 = mwatt_1 = 0

        print(PRINT_FORMAT_2 % (freq, torque_a, torque_b, throttle, volt_0, mamp_0/1000, mwatt_0/1000, rpms))


    while True:
        pass
