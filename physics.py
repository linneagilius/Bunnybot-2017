from pyfrc.physics.drivetrains import four_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller
        self.controller.add_device_gyro_channel('adxrs450_spi_0_angle')
        self.encoder_position = 0.0

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        fl = hal_data['CAN'][0]['value']
        bl = hal_data['CAN'][1]['value']
        fr = -hal_data['CAN'][2]['value']
        br = -hal_data['CAN'][3]['value']

        self.encoder_position += fl * tm_diff

        hal_data['CAN'][0]['enc_position'] += hal_data['CAN'][0]['value'] / \
            1023 * tm_diff * 5000

        rotation, speed = four_motor_drivetrain(bl, br, fl, fr, 3, 0.025)

        self.controller.drive(speed, rotation * 0.75, tm_diff)
