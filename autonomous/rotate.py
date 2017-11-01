from magicbot import AutonomousStateMachine, state
from components.drivetrain import Drivetrain


class Rotate(AutonomousStateMachine):
    MODE_NAME = "Rotate"

    drivetrain = Drivetrain

    @state(first=True)
    def rotate(self):
        if self.drivetrain.rotate(degrees=-90):
            self.done()
