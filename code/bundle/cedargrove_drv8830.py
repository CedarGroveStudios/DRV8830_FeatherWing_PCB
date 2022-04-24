# SPDX-FileCopyrightText: 2021 Cedar Grove Studios
# SPDX-License-Identifier: MIT

"""
`cedargrove_drv8830`
==========================
2021-08-31 v0.1

Simple control of a DC motor using the Cedar Grove Power Monitor / DRV8830 Controller
FeatherWing.

* Author(s): Cedar Grove Studios
"""

__version__ = "0.1.0"
__repo__ = "https://github.com/CedarGroveStudios/Motor_Tester_DRV8830_Wing"

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bit import ROBit

# DEVICE REGISTER MAP
_CONTROL = 0x00  # Control Register      -W
_FAULT = 0x01  # Fault Status Register R-


class VoltageAdapter:
    """Datasheet formula modified to match the datasheet's voltage table. The
    algorithm was inspired by Pimironi's original VoltageAdapter class code,
    Copyright (c) 2018 Pimoroni Ltd. (https://github.com/pimoroni/drv8830-python)."""

    def _index_to_voltage(self, index):
        index = min(max(index, 0), 0x3F)
        if index <= 5:
            return 0.0
        offset = 0.01 if index >= 16 else 0
        offset += 0.01 if index >= 48 else 0
        return round(offset + (index * 0.08), 2)

    def _voltage_to_index(self, volts):
        volts = min(max(volts, 0.0), 5.06)
        if volts < 0.48:
            return 0
        offset = 0.01 if volts >= 1.29 else 0
        offset -= 0.01 if volts >= 3.86 else 0
        return int(offset + volts / 0.08)


class BridgeControl:
    # Bit order: IN2 IN1
    STANDBY = 0b00  # Standby/Coast function (Hi-Z)
    COAST = 0b00  # Standby/Coast function (Hi-Z)
    FORWARD = 0b01  # Forward function
    REVERSE = 0b10  # Reverse function
    BRAKE = 0b11  # Brake function

    DESCRIPTOR = ["STANDBY/COAST", "FORWARD", "REVERSE", "BRAKE"]


class Faults:
    """Fault Register Flag Descriptors
    FAULT  Any fault condition
    OCP    Overcurrent event;
           device disabled, clear fault to reactivate
    UVLO   Undervoltage lockout; device disabled,
           resumes with voltage restoration
    OTS    Overtemperature condition;
           device disabled, resumes with lower temperature
    ILIMIT Extended current limit event;
           device disabled, clear fault to reactivate
    """

    DESCRIPTOR = ["FAULT", "OCP", "UVLO", "OTS", "ILIMIT"]


class DRV8830:
    """DC motor driver with I2C interface. Using an internal PWM scheme, the
    DRV8830 produces a regulated output voltage from a normalized input value
    (-1.0 to +1.0) or voltage input value (-5.06 to +5.06 volts).

    :param i2c_bus: The microcontroller I2C interface bus pins.
    :param address: The I2C address of the DRV8830 motor controller."""

    def __init__(self, i2c_bus, address=0x60):
        """Instantiate DRV8830. Set output voltage to 0.0, place into STANDBY
        mode, and reset all fault status flags."""
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._vset = 0x00
        self._in_x = BridgeControl.STANDBY
        self._clear = True  # Clear all fault status flags
        return

    # DEFINE I2C DEVICE BITS, NYBBLES, BYTES, AND REGISTERS
    _in_x = RWBits(2, _CONTROL, 0, 1, False)  # Output state; IN2, IN1
    _vset = RWBits(6, _CONTROL, 2, 1, False)  # DAC output voltage (raw)
    _fault = ROBit(_FAULT, 0, 1, False)  # Any fault condition
    _ocp = ROBit(_FAULT, 1, 1, False)  # Overcurrent event
    _uvlo = ROBit(_FAULT, 2, 1, False)  # Undervoltage lockout
    _ots = ROBit(_FAULT, 3, 1, False)  # Overtemperature condition
    _ilimit = ROBit(_FAULT, 4, 1, False)  # Extended current limit event
    _clear = RWBit(_FAULT, 7, 1, False)  # Clears fault status flag bits

    @property
    def throttle(self):
        """Current motor speed, ranging from -1.0 (full speed reverse) to
        +1.0 (full speed forward), or ``None`` (controller off). If ``None``,
        the H-bridge is set to high-impedance (coasting). If ``0.0``, the
        H-bridge is set to cause braking."""
        if self.bridge_control[0] == BridgeControl.COAST:
            return None
        if self.bridge_control[0] == BridgeControl.BRAKE:
            return 0.0
        if self.bridge_control[0] == BridgeControl.REVERSE:
            return -1 * round(self._vset / 0x3F, 2)
        return round(self._vset / 0x3F, 2)

    @throttle.setter
    def throttle(self, value):
        if value == None:
            self._vset = 0
            self._in_x = BridgeControl.COAST
            return
        self._throttle_normalized = min(max(value, -1.0), +1.0)  # Constrain value
        if value < 0:
            self._vset = int(abs(value * 0x3F))
            self._in_x = BridgeControl.REVERSE
        elif value > 0:
            self._vset = int(value * 0x3F)
            self._in_x = BridgeControl.FORWARD
        else:
            self._vset = 0
            self._in_x = BridgeControl.BRAKE
        return

    @property
    def throttle_volts(self):
        """Current motor speed, ranging from -5.06 volts (full speed reverse) to
        +5.06 volts (full speed forward), or ``None`` (controller off). If ``None``,
        the H-bridge is set to high-impedance (coasting). If ``0.0``, the
        H-bridge is set to cause braking."""
        if self.bridge_control[0] == BridgeControl.COAST:
            return None
        if self.bridge_control[0] == BridgeControl.BRAKE:
            return 0.0
        if self.bridge_control[0] == BridgeControl.REVERSE:
            return -1 * VoltageAdapter._index_to_voltage(self, self._vset)
        return VoltageAdapter._index_to_voltage(self, self._vset)

    @throttle_volts.setter
    def throttle_volts(self, value):
        if value == None:
            self._vset = 0
            self._in_x = BridgeControl.COAST
            return
        value = min(max(value, -5.1), +5.1)  # constrain value
        if value < 0:
            self._vset = VoltageAdapter._voltage_to_index(self, abs(value))
            self._in_x = BridgeControl.REVERSE
        elif value > 0:
            self._vset = VoltageAdapter._voltage_to_index(self, value)
            self._in_x = BridgeControl.FORWARD
        else:
            self._vset = 0
            self._in_x = BridgeControl.BRAKE
        return

    @property
    def bridge_control(self):
        """Motor driver bridge status. Returns the 2-bit bridge control integer
        value and corresponding description string."""
        return self._in_x, BridgeControl.DESCRIPTOR[self._in_x]

    @property
    def fault(self):
        """Motor driver fault register status. Returns state of FAULT flag and
        a list of activated fault flag descriptors. FAULT flag is ``True`` if
        one or more fault register flags are ``True``."""
        faults = []
        if self._fault:
            faults.append(Faults.DESCRIPTOR[0])
            if self._ocp:
                faults.append(Faults.DESCRIPTOR[1])
            if self._uvlo:
                faults.append(Faults.DESCRIPTOR[2])
            if self._ots:
                faults.append(Faults.DESCRIPTOR[3])
            if self._ilimit:
                faults.append(Faults.DESCRIPTOR[4])
        return self._fault, faults

    def clear_faults(self):
        """Clears all fault conditions."""
        self._clear = True  # Clear all fault status flags
        return

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self._vset = 0
        self._in_x = BridgeControl.STANDBY
