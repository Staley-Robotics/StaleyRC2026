import wpilib

from phoenix6.hardware import TalonFX
from ntcore.util import ntproperty

class MyRobot(wpilib.TimedRobot):
    
    # speed = ntproperty("/speed", defaultValue=0, persistent=True)

    def robotInit(self):
        super().robotInit()
        agitator = TalonFX(12)
        launcher = TalonFX(13)