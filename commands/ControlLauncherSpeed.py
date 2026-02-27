import typing

from commands2 import Command, Subsystem
from wpimath.units import percent

from subsystems import Flywheel, LauncherConstants

class ControlLauncherSpeed(Command):
    # Variable Declaration
    launcher_sys:Flywheel = None
    
    # Initialization
    def __init__( self,
                  launcherSys:Subsystem,
                  speedInput:typing.Callable[[], percent]=lambda:0.0
                ) -> None:
        # Command Attributes
        self.launcher_sys:Flywheel = launcherSys
        self.speedInput:typing.Callable[[], percent] = speedInput

        self.setName( f"ControlLauncherSpeed" )
        self.addRequirements( launcherSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.launcher_sys.setDesiredSpeed(
            self.speedInput() * LauncherConstants.kMaxExpectedSpeed
        )

    def end(self, interrupted:bool) -> None:
        self.launcher_sys.setIntakeSpeed(0.0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False