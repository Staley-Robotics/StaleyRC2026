from phoenix6 import hardware, controls, configs, signals
from wpilib import TimedRobot, XboxController

class MyRobot(TimedRobot):
    def __init__(self, period = 0.02):
        super().__init__(period)
        self.controller = XboxController(0)
        self.iMotor = hardware.TalonFX(0, "rio")

    def teleopPeriodic(self):
        trigger = self.controller.getRightTriggerAxis()
        power = trigger * 0.5
        self.iMotor.set(power)