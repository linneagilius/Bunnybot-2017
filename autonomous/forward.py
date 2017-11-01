from magicbot import AutonomousStateMachine, state
from components.drivetrain import Drivetrain


class Forward(AutonomousStateMachine):
    MODE_NAME = "Forward"
    DEFAULT = True

    drivetrain = Drivetrain

    @state(first=True)
    def forward(self):
        if self.drivetrain.forward(feet=25):
            self.done()
