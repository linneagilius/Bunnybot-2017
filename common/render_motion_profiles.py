#!/usr/bin/env python3
"""Render MotionProfiles using matplotlib

This is useful for testing to see the big picture
 view of the MotionProfile
"""
import os
import errno

import matplotlib.pyplot
import numpy

from motion_profiles import MotionProfile


def get_velocity(motion_profile: MotionProfile, time: float):
    sign = -1 if motion_profile.reverse else 1
    if time <= motion_profile.acceleration_time:
        return motion_profile.acceleration * time * sign
    elif time <= motion_profile.deceleration_time:
        return motion_profile.max_speed * sign
    else:
        max_speed = (
            motion_profile.acceleration * motion_profile.acceleration_time)
        return (max_speed + (motion_profile.deceleration *
                             (time - motion_profile.deceleration_time))) * sign


def plot_profile(profile, file_name):
    time = numpy.arange(0, profile.end_time, 0.005)
    position = list(map(lambda time: profile.position(time), time))
    velocity = list(map(lambda time: get_velocity(profile, time), time))
    matplotlib.pyplot.plot(time, position, "C0")
    matplotlib.pyplot.plot(time, velocity, "C2")
    matplotlib.pyplot.savefig(file_name)
    matplotlib.pyplot.close()


# Can be run as stand-alone module, will generate graphs
#  of two test motion profiles and save them as images
if __name__ == "__main__":
    try:
        os.makedirs('profiles')
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    triangular_profile = MotionProfile.generate_motion_profile(
        4.5, 9 / (2.15), 9, 35.69)
    plot_profile(triangular_profile, 'profiles/triangular_profile.png')
    trapezoidal_profile = MotionProfile.generate_motion_profile(3, 2, 6, 24)
    plot_profile(trapezoidal_profile, 'profiles/trapezoidal_profile.png')
    negative_profile = MotionProfile.generate_motion_profile(
        4.5, 4, 9, -30)
    plot_profile(negative_profile, 'profiles/negative_profile.png')
