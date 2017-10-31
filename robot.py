#!/usr/bin/env python3

import wpilib
from ctre.cantalon import CANTalon
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
from components.bunny_grabber import BunnyGrabber
from components.flipper import Flipper


class Robot(MagicRobot):

    drivetrain = Drivetrain
    bunnygrabber = BunnyGrabber
    flipper = Flipper

    def createObjects(self):
        self.robot_drive = wpilib.RobotDrive(
            0, 1, 2, 3, motorController=CANTalon)
        self.bunny_motor = wpilib.Talon(0)

        self.flipper_motor = wpilib.Talon(1)

        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

    def teleopPeriodic(self):
        self.drivetrain.turn_at(
            self.drive_joystick.getRawAxis(0), squaredInputs=True)
        self.drivetrain.forward_at(self.drive_joystick.getRawAxis(1))

        if self.drive_joystick.getRawButton(4):
            self.bunny_grabber.set_forward()
        elif self.drive_joystick.getRawButton(5):
            self.bunny_grabber.set_backward()

        if self.drive_joystick.getRawButton(1):
            self.flipper.turn_on()
        else:
            self.flipper.turn_off()


if __name__ == '__main__':
    wpilib.run(Robot)
