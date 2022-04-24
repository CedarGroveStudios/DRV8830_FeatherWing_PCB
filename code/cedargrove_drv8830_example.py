# cedargrove_DRV8830_example.py
# with Cedar Grove DRV8830 I2C Motor Controller FeatherWing
# 2022-02-07 v02 Cedar Grove Studios

import board
import analogio
import time
import cedargrove_drv8830
#import busio
from simpleio import map_range

print("imports completed")


"""# Test for I2C device
import busio
i2c = board.I2C()
while not i2c.try_lock():
    pass
while True:
    print(i2c.scan(), [hex(x) for x in i2c.scan()], [bin(x) for x in i2c.scan()])
    time.sleep(1)
    pass"""


# initialize analog current inputs
# ina271 output is 20x the voltage across the 0.100-ohm shunt resistor
#   3.3v = 1.650A
current_monitor_test = analogio.AnalogIn(board.A4)
current_monitor_brake = analogio.AnalogIn(board.A5)
print("analog inputs defined")

i2c = board.I2C()
i2c.deinit()

i2c = board.I2C()

print("i2c = board.I2C()")
drv8830_test = cedargrove_drv8830.DRV8830(i2c)
print("first (test) board initialized")

drv8830_brake = cedargrove_drv8830.DRV8830(i2c, address=0x61)
print("second (brake) board initialized")

STEP = 6
direction = 1  # forward
drv8830_test.clear_faults
drv8830_brake.clear_faults
while True:
    if drv8830_test.fault[0]:
        print("*** fault: test motor ***", drv8830_test.fault[1])
        time.sleep(5)
    if drv8830_brake.fault[0]:
        print("*** fault: test motor ***", drv8830_test.fault[1])
        time.sleep(5)

    for i in range(0, 101, STEP):
        drv8830_test.throttle = i / 100 * direction
        drv8830_brake.throttle = i / 100 * direction
        print(
            f" {drv8830_test.bridge_control[1]:14s} step:{i:3.0f}  throttle:{drv8830_test.throttle:+3.2f}  {drv8830_test.throttle_volts:+3.2f} volts"
        )
        print(((map_range(current_monitor_test.value, 0, 65535, 0, 1650), drv8830_test.throttle * 100, map_range(current_monitor_brake.value, 0, 65535, 0, 1650))))
        time.sleep(0.1)
    time.sleep(1)
    for i in range(100, -1, -1 * STEP):
        drv8830_test.throttle = i / 100 * direction
        drv8830_brake.throttle = i / 100 * direction
        print(
            f" {drv8830_test.bridge_control[1]:14s} step:{i:3.0f}  throttle:{drv8830_test.throttle:+3.2f}  {drv8830_test.throttle_volts:+3.2f} volts"
        )
        print(((map_range(current_monitor_test.value, 0, 65535, 0, 1650), drv8830_test.throttle * 100, map_range(current_monitor_brake.value, 0, 65535, 0, 1650))))
        time.sleep(0.1)
    direction = direction * -1
    time.sleep(2)
    drv8830_test.throttle = None
    drv8830_brake.throttle = None
    print(" COAST")
    time.sleep(1)
