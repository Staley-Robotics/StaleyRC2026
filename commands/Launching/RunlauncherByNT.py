import typing

from commands2 import Command, Subsystem
from wpimath.units import percent
from phoenix6.units import rotations_per_second
from ntcore.util import ntproperty

from subsystems import Launcher

class RunLauncherByNT(Command):
    # Variable Declaration
    flywheel_sys:Launcher = None

    speed = ntproperty("/Settings/RunLauncherByNT/speed (rps: 0-70)", 0.0, persistent=True)
    
    # Initialization
    def __init__( self,
                  flywheelSys:Launcher,
                ) -> None:
        # Command Attributes
        self.flywheel_sys:Launcher = flywheelSys

        self.setName( f"RunLauncherByNT" )
        self.addRequirements( flywheelSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.flywheel_sys.setDesiredSpeed( self.speed )

    def end(self, interrupted:bool) -> None:
        self.flywheel_sys.setDesiredSpeed( 0.0 )
    
    def changeSpeed(self, speedModification:rotations_per_second) -> None:
        self.speed += speedModification

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False