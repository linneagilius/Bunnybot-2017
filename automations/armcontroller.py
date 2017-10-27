from components.bunnygrabber import BunnyGrabber
from components.armextention import ArmExtention


class ArmController:
    """
    Controls all movement on the arm.
    """
    grabber = BunnyGrabber
    extension = ArmExtention

    def set_extended(self, extended):
        self.extension.set_extended(extended)

    def grabber_intake(self):
        self.grabber.set_forward()

    def grabber_release(self):
        self.grabber.set_backward()
