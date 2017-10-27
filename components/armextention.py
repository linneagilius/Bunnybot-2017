import wpilib


class ArmExtention:
    """
    Controls movement of the bunny grabber arm.
    """

    encoder = wpilib.AnalogPotentiometer
    motor = wpilib.Talon
    stop_value = 0
    extended = False

    def set_stop_value(self, stop_value):
        self.stop_value = stop_value

    def set_extended(self, extended):
        self.extended = extended

    def execute(self):
        if self.encoder.get() > self.stop_value:  # if the arm is overextended
            self.motor.set(-0.05)  # move arm back slowly
        else:
            if self.extended:  # if we want the arm to be extended
                self.motor.set(0.2)  # move arm forwards
            elif self.encoder.get() > 0:  # if we don't want the arm to be extended but it is
                self.motor.set(-0.2)  # move arm back
            elif self.encoder.get() < 0:  # if the arm is underextended
                self.motor.set(0.05)  # move arm forwards slowly
