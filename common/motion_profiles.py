#Provide motion profiles for smooth and efficient motion

import math
from typing import NamedTuple, Callable

import wpilib

from common.pid import PIDCoefficients, PIDController


class MotionProfile(NamedTuple):
    #Motion profile representing a specific desired move
    #Robot's acceleration - slope of acceleration triangle
    acceleration: float
    #Robot's deceleration - slope of deceleration triangle
    deceleration: float
    #Robot's max speed
    max_speed: float
    #Time to stop acceleration
    acceleration_time: float
    #Time to start decelerating
    deceleration_time: float
    #Time when motion profile should be finished
    end_time: float
    #Boolean flag for reverse, or negative direction
    reverse: bool

    @staticmethod
    def generate_motion_profile(acceleration_time: float,
                                deceleration_time: float, 
                                max_speed: float,
                                target_distance: float):
        """
        Create a motion profile to efficiently travel `target_distance`
        using the specified parameters
        'acceleration_time`: Time it takes for robot to accelerate from rest
        to maximum speed.
        'deceleration_time`: Time it takes for robot to decelerate from
        maximum speed to rest.
        'max_speed`: The maximum speed the robot can go.
        target_distance`: The distance the robot should travel using the
        motion profile.
        ** Note: The units don't matter, as long as they are all the same
         - but anyone caught using non-metric units will be publicly shamed.**
        """
        
        # Handle negative distances
        reverse = (target_distance < 0.0)
        target_distance = abs(target_distance)

        # How far it takes the robot to accelerate from rest to max speed
        # Corresponds to area under acceleration triangle
        full_acceleration_distance = 0.5 * acceleration_time * max_speed
        # Acceleration of the robot
        acceleration = max_speed / acceleration_time
        # How far it takes the robot to decelerate from max speed to rest
        # Corresponds to area under deceleration triangle
        full_deceleration_distance = 0.5 * deceleration_time * max_speed
        # Deceleration of the robot
        deceleration = -max_speed / deceleration_time

        # Total distance needed to reach max speed from rest,
        #  and then return back to rest
        distance_for_max_speed = (
            full_acceleration_distance + full_deceleration_distance)

        # If the target distance is greater than the distance it takes
        #  for the robot to accelerate to max speed and back to rest
        #  the motion profile will be trapezoidal
        if distance_for_max_speed - target_distance <= 0.0:
            # The time the robot will spend at full speed
            full_speed_time = (
                target_distance - distance_for_max_speed) / max_speed
            # Time at which robot should transition from acceleration
            #  to constant max speed
            end_acceleration_time = acceleration_time
            # Time when robot should start decelerating to rest
            start_deceleration_time = acceleration_time + full_speed_time
            # The time at which the profile will be completed
            end_time = start_deceleration_time + deceleration_time
            return MotionProfile(acceleration, deceleration, max_speed,
                                 end_acceleration_time,
                                 start_deceleration_time, end_time, reverse)

        # If it can't accelerate all the way to max speed and back
        #  without overshooting, the motion profile will be triangular
        else:
            
            """
            Distance to get to the top speed the robot will reach - since
            it doesn't have time to accelerate fully this is not the
            same as full_acceleration_distance.
            This is calculated using the fact that the acceleration and
            deceleration distances for a triangular motion profile are
            directly proportional to the ratio of the acceleration and
            deceleration times
            """
            
            partial_acceleration_distance = target_distance * \
                (acceleration_time / (acceleration_time + deceleration_time))
            # Time to get to the top speed the robot will actually reach
            # x = 0.5at^2 -> t = sqrt(2x/a)
            partial_acceleration_time = math.sqrt(
                (2 * partial_acceleration_distance) / acceleration)
            # The actual maximum speed the robot will be able to reach
            #  before it needs to begin deceleration
            partial_max_speed = partial_acceleration_time * acceleration
            # The time it will take the robot to declerate from
            #  partial_max_speed to rest - how much time will the
            #  robot need to stop
            partial_deceleration_time = -partial_max_speed / deceleration
            # The time at which the motion profile should be completed
            end_time = partial_acceleration_time + partial_deceleration_time
            # Time to stop accelerating
            acceleration_end_time = partial_acceleration_time
            # Time to start decelerating
            deceleration_start_time = end_time - partial_deceleration_time
            motion_profile = MotionProfile(
                acceleration, deceleration, partial_max_speed,
                acceleration_end_time, deceleration_start_time,
                end_time, reverse)
            return motion_profile
        
    """
    position() can be used for times after the end of the profile,
    this is so that if the PID hasn't got the robot to the
    target position by the end it can keep reducing the position
    error till it reaches the target position
    """
    
    def position(self, time):
        #Get the optimal position at a specific time
        # Inner helper function to find the distance traveled
        #  when accelerating from an initial velocity for a time period
        def distance(initial_speed, acceleration, time):
            #find the distance traveled when accelerating from
            #'initial_speed' at 'acceleration' for 'time'
           
            # delta_x = vi + 1/2(at^2)
            return (initial_speed * time) + (0.5 * acceleration * time * time)

        position = 0
        # Acceleration phase - time is clamped between zero and
        #  acceleration time
        acceleration_phase_time = max(0, min(time, self.acceleration_time))
        position += distance(0.0, self.acceleration, acceleration_phase_time)

        # Max speed phase - time is clamped between 0 and time change between
        #  acceleration and deceleration
        max_speed_phase_time = max(
            0, min(time, self.deceleration_time) - self.acceleration_time)
        position += max_speed_phase_time * self.max_speed

        # Deceleration phase - time is clamped between zero and the time
        #  change between deceleration and the end of the motion profile
        deceleration_phase_time = min(
            max(0, time - self.deceleration_time),
            self.end_time - self.deceleration_time)
        position += distance(self.max_speed, self.deceleration,
                             deceleration_phase_time)

        # Handle reverse (negative) directions
        return position if not self.reverse else -position


class ProfileExecutor:
    def __init__(self, pid_coefs: PIDCoefficients,
                 motion_profile: MotionProfile,
                 input_source: Callable[[], float],
                 output: Callable[[float], None],
                 acceptable_error_margin: float):
        
        
        """Wrapper for a PID controller and a motion profile. Ties
         them together for seemless profile execution
        Uses `input_source` to retrieve current input for motion profile,
         and `output` to write PID output. `acceptable_error_margin` is the
         acceptable amount of error as a decimal.
        """

        self.pid = PIDController(pid_coefs, 1.0, -1.0)
        self.profile_start_time = wpilib.Timer.getFPGATimestamp()
        self.motion_profile = motion_profile
        self.input_source = input_source
        self.output = output
        self.acceptable_error_margin = acceptable_error_margin

    def update(self) -> bool:
        """Updates motion profile and writes output. 
        Returns 'True' if profile is completed 
        (robot is within error margin of target), otherwise `False`.
        """
        
        time_delta = wpilib.Timer.getFPGATimestamp() - self.profile_start_time

        current_goal_position = self.motion_profile.position(time_delta)

        current_input = self.input_source()

        output = self.pid.get_output(current_input, current_goal_position)
        self.output(output)
        final_position = self.motion_profile.position(
            self.motion_profile.end_time)

        error_margin = abs(final_position - current_input) / \
            abs(final_position)
        return error_margin < self.acceptable_error_margin
