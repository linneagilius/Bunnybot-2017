'''This module provide functionality related to PID controllers

namedtuple for stroing PID coefficients
PID controller object
'''

from typing import NamedTuple

from wpilib import Timer
from utils import clamp


class PIDCoefficients(NamedTuple):
    '''Coefficients for PID controller'''
    p: float
    i: float
    d: float


class PIDController:
    '''PID controller for a process.

    See https://en.wikipedia.org/wiki/PID_controller for
    overview/control algorithm

    Supports both continuous and non-continuous input/setpoint
    ranges. For example, controlling motor speed is a
    non-continuous process, while position of a motor would be
    continuous as the position "wraps around" at some point.
    '''

    def __init__(self,
                 pid_coefs: PIDCoefficients,
                 output_max: float = None,
                 output_min: float = None,
                 input_max: float = None,
                 input_min: float = None,
                 continuous: bool = False):
        '''PID controller constructor

        p:
            Proportional coefficient
        i:
            Integral coefficient
        d:
            Derivative coefficient
        continuous:
            Flag for whether the setpoint/input range is continuous
        min_input, max_input:
            Set min/max of the setpoint/input range, only
            needed if continuous is set to True
        '''

        # PID control coefficients
        self._coefs = pid_coefs

        # Internal variables used in the control alogorithm
        self._integral_term = 0.0
        # System time in seconds at last update
        self._previous_time = 0.0
        # PID input at last update
        self._previous_input = 0.0

        # Whether the setpoint/input range is continous
        self._continuous = continuous
        # Range of input/setpoint values to expect
        self._input_max = input_max
        self._input_min = input_min
        # Acceptable output range
        self._output_max = output_max
        self._output_min = output_min

    def get_output(self, current_input: float, setpoint: float) -> float:
        '''Get PID output for process

        current_input:
            The current PID input
        setpoint:
            Desired output of process/input to PID
        '''

        # Current time in seconds
        current_time = Timer.getFPGATimestamp()

        # Time elapsed since last update
        time_change = current_time - self._previous_time

        # The current error
        current_error = self._get_continuous_error(setpoint - current_input)

        self._integral_term += self._coefs.i * (current_error * time_change)
        self._integral_term = clamp(self._integral_term, self._output_max,
                                    self._output_min)

        # Protect againsts ZeroDivisionError caused
        #  by time resolution in simulator
        if time_change <= 0.0:
            time_change = 0.005

        derivative = (current_input - self._previous_input) / time_change

        self._previous_input = current_input
        self._previous_time = current_time

        output = ((self._coefs.p * current_error) + self._integral_term +
                  (self._coefs.d * derivative))
        return clamp(output, self._output_max, self._output_min)

    def reset(self):
        '''Reset internal control variables

        Should be used if the PID is disabled for a time,
        then re-enabled.
        '''
        self._previous_input = 0.0
        self._integral_term = 0.0
        self._previous_time = 0.0

    def _get_continuous_error(self, error):
        if self._continuous:
            if abs(error) > (self._input_max - self._input_min) / 2.0:
                if error > 0.0:
                    return error - (self._input_max - self._input_min)
                return error + (self._input_max - self._input_min)
        return error
