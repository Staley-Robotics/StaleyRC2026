import wpilib

from phoenix6.hardware import TalonFX
from ntcore.util import ntproperty

from util import FalconXboxController

class MyRobot(wpilib.TimedRobot):
    
    # speed = ntproperty("/speed", defaultValue=0, persistent=True)

    def robotInit(self):
        super().robotInit()
        self.controller = FalconXboxController(0)
    
    def robotPeriodic(self):
        super().robotPeriodic()

        if self.controller.a().getAsBoolean():
            print('hi')

            iters = 1000

            times = []

            x = 0

            time = wpilib.getTime()
            for i in range(iters):
                x = x+1

            times.append(wpilib.getTime() - time)
            print(f"increment: {times[-1]}")

            time = wpilib.getTime()
            for i in range(iters):
                x = wpilib.DriverStation.getAlliance()

            times.append(wpilib.getTime() - time)
            print(f"ds.alliance: {times[-1]}")

            print(f'pct diff: {times[1]/times[0]}')

