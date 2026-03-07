import typing

from commands2 import Command
from wpimath.units import percent

from subsystems import Launcher

class RunLauncherByDist(Command):
    # Variable Declaration
    launcher_sys:Launcher = None
    
    # Initialization
    def __init__( self,
                  launcherSys:Launcher,
                ) -> None:
        # Command Attributes
        self.launcher_sys:Launcher = launcherSys

        self.setName( f"ControlLauncherSpeed" )
        self.addRequirements( launcherSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # get dist from center of launcher to center of fuel place
        dist = None

        # scale dist to percent between min dist and max dist
        dist = None

        # apply scale between speed at min dist and max dist, by some math magic
        speed = self.launcher_sys.LauncherSpeeds.SPEED_AT_ZERO_DIST

        # apply speed
        self.launcher_sys.setDesiredSpeed(
            speed
        )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False