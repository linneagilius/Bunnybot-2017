import wpilib


class BunnyGrabber:
    """Controls the bunny grabber.
    """
    bunny_motor = wpilib.Talon
    direction = 0

    def set_backward(self):
        self.direction = -1

    def set_forward(self):
        self.direction = 1

    def execute(self):
        if self.direction == 1:
            self.bunny_motor.setSpeed(0.5)
        elif self.direction == -1:
            self.bunny_motor.setSpeed(-0.5)
        else:
            self.bunny_motor.setSpeed(0)
        self.direction = 0
