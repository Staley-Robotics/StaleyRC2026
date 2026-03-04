import typing

from commands2 import Command, Subsystem
from wpimath.units import percent

from subsystems import Launcher

class RunLauncherByDist(Command):
    # Variable Declaration
    launcher_sys:Launcher = None
    
    # Initialization
    def __init__( self,
                  launcherSys:Subsystem,
                ) -> None:
        # Command Attributes
        self.launcher_sys:Launcher = launcherSys

        self.setName( f"ControlLauncherSpeed" )
        self.addRequirements( launcherSys )

    def initialize(self) -> None:
        self.launcher_sys.setDesiredSpeed(
            self.launcher_sys.LauncherSpeeds.SPEED_AT_ZERO_DIST
        )

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False