import typing

from commands2 import Command, Subsystem
from wpimath.units import percent

from subsystems import Launcher

class LauncherDefault(Command):
    '''
    Keeps the launcher spinning at its 'wait' speed to reduce acceleration times when needed
    '''
    # Variable Declaration
    launcher_sys:Launcher = None
    
    # Initialization
    def __init__( self,
                  launcherSys:Subsystem,
                ) -> None:
        # Command Attributes
        self.launcher_sys:Launcher = launcherSys

        self.setName( f"LauncherDefault" )
        self.addRequirements( launcherSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.launcher_sys.setDesiredSpeed(
            self.launcher_sys.LauncherSpeeds.WAIT
        )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False