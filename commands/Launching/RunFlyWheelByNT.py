import typing

from commands2 import Command, Subsystem
from wpimath.units import percent
from ntcore.util import ntproperty

from subsystems import Launcher, Agitator

class RunFlyWheelByNT(Command):
    # Variable Declaration
    flywheel_sys:Launcher|Agitator = None

    speed = ntproperty("/Settings/RunFlywheelByNT/speed (rps: 0-70)", 0.0, persistent=True)
    
    # Initialization
    def __init__( self,
                  flywheelSys:Launcher|Agitator,
                ) -> None:
        # Command Attributes
        self.flywheel_sys:Launcher|Agitator = flywheelSys

        self.setName( f"RunFlywheelByNT - {flywheelSys.__class__.__name__}" )
        self.addRequirements( flywheelSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.flywheel_sys.setDesiredSpeed( self.speed )

    def end(self, interrupted:bool) -> None:
        self.flywheel_sys.setDesiredSpeed( 0.0 )

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False