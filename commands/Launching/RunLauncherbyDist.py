import typing

from commands2 import Command
from wpimath.units import percent

from ntcore.util import ntproperty

from subsystems import Launcher
from util import RebuiltCalc

class RunLauncherByDist(Command):
    # Variable Declaration
    launcher_sys:Launcher = None

    a = ntproperty("Settings/RunLauncherByDist/mult a (dist)", 1.0, persistent=True)
    b = ntproperty("Settings/RunLauncherByDist/mult b (dist^2)", 0.2, persistent=True)
    c = ntproperty("Settings/RunLauncherByDist/const", 14.0, persistent=True)
    
    # Initialization
    def __init__( self,
                  launcherSys:Launcher,
                ) -> None:
        # Command Attributes
        self.launcher_sys:Launcher = launcherSys

        self.setName( f"RunLauncherByDist" )
        self.addRequirements( launcherSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # get dist from center of launcher to center of fuel place
        dist = RebuiltCalc.getDistToTarget()

        # scale dist to percent between min dist and max dist
        # pct_of_dist = (dist-Launcher.LauncherDistances.MIN)/(Launcher.LauncherDistances.MAX-Launcher.LauncherDistances.MIN) # scales to percentage between

        # apply scale between speed at min dist and max dist, by some math magic
        speed = (self.a*dist) + (self.b*(dist*dist)) + self.c

        # apply speed
        self.launcher_sys.setDesiredSpeed(
            speed
        )

    def end(self, interrupted:bool) -> None:
        self.launcher_sys.setDesiredSpeed(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
    
    def change_c(self, change_by:float) -> None:
        self.c += change_by