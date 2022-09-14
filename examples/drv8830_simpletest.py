# SPDX-FileCopyrightText: Copyright (c) 2022 JG for Cedar Grove Maker Studios
#
# SPDX-License-Identifier: Unlicense

# cedargrove_DRV8830_simpletest.py

import time
import board
import cedargrove_drv8830

# Instantiate motor controller; clear any faults
motor_1 = cedargrove_drv8830.DRV8830(board.I2C())
motor_1.clear_faults()

direction = 1  # Set initial direction: forward; -1 for reverse

while True:
    print("motor bridge_control, throttle, volts")
    # Accelerate from stop to full speed
    for i in range(0, 101, 5):
        motor_1.throttle = i / 100 * direction
        print(motor_1.bridge_control[1], motor_1.throttle, motor_1.throttle_volts)
        time.sleep(0.1)
    time.sleep(1)  # Hold at full speed

    # Deaccelerate from full speed to stop
    for i in range(100, -1, -5):
        motor_1.throttle = i / 100 * direction
        print(motor_1.bridge_control[1], motor_1.throttle, motor_1.throttle_volts)
        time.sleep(0.1)

    direction = direction * -1  # Change motor direction

    motor_1.throttle = None  # Coast the motor
    time.sleep(1)  # Pause to settle before continuing
