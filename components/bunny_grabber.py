import wpilib
import enum

__all__ = ["BunnyGrabber"]


class GrabberDirection(enum.Enum):
    intake = 1
    release = -1
    stopped = 0


class BunnyGrabber:
    """Controls the bunny grabber"""
    bunny_motor = wpilib.Talon
    direction = GrabberDirection.stopped

    def set_intake(self):
        self.direction = GrabberDirection.intake

    def set_release(self):
        self.direction = GrabberDirection.release

    def execute(self):
        if self.direction == GrabberDirection.intake:
            self.bunny_motor.setSpeed(0.5)
        elif self.direction == GrabberDirection.release:
            self.bunny_motor.setSpeed(-0.5)
        else:
            self.bunny_motor.setSpeed(0)
        self.direction = GrabberDirection.stopped
