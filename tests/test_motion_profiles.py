"""Test module for motion_profiles.py"""

import pytest

from common.motion_profiles import MotionProfile


class TestMotionProfile:
    """Test class for MotionProfile"""

    def test_generate_motion_profile_reach_max_speed(self):
        """Test MotionProfile.generate_motion_profile() with robot
         reaching max speed - trapezoidal"""
        correct_motion_profile = MotionProfile(2, -3, 6, 3, 4.5, 6.5, False)
        generated_motion_profile = MotionProfile.generate_motion_profile(
            3, 2, 6, 24)

        assert correct_motion_profile == pytest.approx(
            generated_motion_profile)

    def test_generate_motion_profile_no_max_speed(self):
        """Test MotionProfile.generate_motion_profile() where
         the robot should never reach reach max speed - triangular"""
        correct_motion_profile = MotionProfile(
            1.5, -1.2, 4.5, 3, 3, 6.75, False)
        generated_motion_profile = MotionProfile.generate_motion_profile(
            4, 5, 6, 15.1875)

        assert correct_motion_profile == pytest.approx(
            generated_motion_profile)

        correct_motion_profile = MotionProfile(
            2, -2.15, 8.6, 4.3, 4.3, 8.3, False)
        generated_motion_profile = MotionProfile.generate_motion_profile(
            4.5, 9 / (2.15), 9, 35.69)

        assert correct_motion_profile == pytest.approx(
            generated_motion_profile)

    def test_position_calculations_trapezoid(self):
        """Test MotionProfile position() for a trapezoidal profile"""
        motion_profile = MotionProfile.generate_motion_profile(
            3, 2, 6, 24)
        # Test 4 points on trapezoid - 1 during acceleration,
        #  1 during max-speed, and 2 during deceleration
        times = [2, 4, 5, 6]
        correct_positions = [4, 15, 20.625, 23.625]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

    def test_position_calculations_triangle(self):
        """Test MotionProfile position() for a triangular profile"""
        motion_profile = MotionProfile.generate_motion_profile(
            4, 5, 6, 15.1875)
        # Test 4 points on triangle - 2 during acceleration,
        #  2 during deceleration
        times = [1, 2, 4, 5.75]
        correct_positions = [0.75, 3, 10.65, 14.5875]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

    def test_negative_distance(self):
        """Test that MotionProfile handles negative distances correctly"""
        motion_profile = MotionProfile.generate_motion_profile(
            4, 5, 6, -15.1875)
        # Test 4 points on triangle - 2 during acceleration,
        #  2 during deceleration
        times = [1, 2, 4, 5.75]
        correct_positions = [-0.75, -3, -10.65, -14.5875]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

        motion_profile = MotionProfile.generate_motion_profile(
            3, 2, 6, -24)
        # Test 4 points on trapezoid - 1 during acceleration,
        #  1 during max-speed, and 2 during deceleration
        times = [2, 4, 5, 6]
        correct_positions = [-4, -15, -20.625, -23.625]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)
