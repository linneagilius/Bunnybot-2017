import math

import wpilib
import ctre

from common.motion_profiles import MotionProfile, ProfileExecutor
from common.pid import PIDCoefficients

# Only the Drivetrain needs to be used outside of this module,
#  if we need to expose something else later we can
__all__ = ["Drivetrain"]


class Drivetrain:
    robot_drive = wpilib.RobotDrive
    gyro = wpilib.ADXRS450_Gyro
    fl_motor = ctre.CANTalon

    def __init__(self):
        self.rotation = 0
        self.forward_speed = 0
        self.gyro_offset = 0.0
        self.profile_executor = None
        self.profile_arguments = None

    def forward_at(self, speed):
        self.forward_speed = speed

    def turn_at(self, speed, squaredInputs=False):
        self.rotation = speed
        if squaredInputs:
            self.rotation = speed**2 if speed >= 0 else -(speed**2)

    def forward(self, feet=0, inches=0, meters=0):
        """Use a motion profile and PID control to efficiently
         move the robot the specified distance forward

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile."""
        # 1 inch = 0.0254 meters
        # 1 foot = 0.3048 meters
        distance = (inches * 0.0254) + (feet * 0.3048) + meters

        # When called with the same arguments, update executor
        if distance == self.profile_arguments:
            return self._update_executor()

        # When target is switched without calling
        #  cancel_motion_profile, issue warning
        if self.profile_arguments is not None:
            print("Use Drivetrain.cancel_motion_profile to change profile!",
                  "Switching to new profile...")

        self.profile_arguments = distance

        motion_profile = MotionProfile.generate_motion_profile(
            acceleration_time=1,
            deceleration_time=1,
            max_speed=1,
            target_distance=distance)

        coefs = PIDCoefficients(p=1.0, i=0.0, d=0.0)

        # Set current position to zero
        self._reset_encoder_position()

        self.profile_executor = ProfileExecutor(
            coefs, motion_profile,
            lambda: self._get_encoder_position() / 1023,
            lambda output: self.forward_at(-output), 0.08)

        return False

    def backward(self, feet=0, inches=0, meters=0):
        """Use a motion profile and PID control to efficiently
         move the robot the specified distance backward

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile."""
        return self.forward(-feet, -inches, -meters)

    def rotate(self, degrees=0):
        """Use a motion profile and PID control to efficiently
         turn the robot the number of degrees - positive is clockwise,
         negative is counter-clockwise

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile."""
        radians = degrees * (math.pi / 180)

        # When called with the same arguments, update executor
        if radians == self.profile_arguments:
            return self._update_executor()

        # When target is switched, issue warning
        if self.profile_arguments is not None:
            print("Use Drivetrain.cancel_motion_profile to change profile!",
                  "Switching to new profile...")

        self.profile_arguments = radians

        motion_profile = MotionProfile.generate_motion_profile(
            acceleration_time=1,
            deceleration_time=1,
            max_speed=0.6,
            target_distance=radians)

        self._zero_gyro()

        coefs = PIDCoefficients(p=1.0, i=0.1, d=0.0)
        self.profile_executor = ProfileExecutor(
            coefs, motion_profile,
            lambda: self._get_gyro_angle(),
            lambda output: self.turn_at(output), 0.001
        )

        return False

    def cancel_motion_profile(self):
        self.profile_executor = None
        self.profile_arguments = None

    def execute(self):
        self.robot_drive.arcadeDrive(self.forward_speed, self.rotation)

        self.rotation = 0
        self.forward_speed = 0

    def _reset_encoder_position(self):
        self.fl_motor.setEncPosition(0)

    def _get_encoder_position(self):
        return -self.fl_motor.getPosition()

    def _zero_gyro(self):
        self.gyro_offset = self.gyro.getAngle()

    def _get_gyro_angle(self):
        return (self.gyro.getAngle() - self.gyro_offset) * (math.pi / 180.0)

    def _update_executor(self):
        executor_finished = self.profile_executor.update()
        if executor_finished:
            self.cancel_motion_profile()
            return True
        return False
