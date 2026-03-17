import typing

from commands2 import Command, Subsystem
from wpimath.units import percent
from phoenix6.units import rotations_per_second

from subsystems import Launcher, Agitator

class SetFlywheelSpeed(Command):
    # Variable Declaration
    flywheel_sys:Launcher = None
    
    # Initialization
    def __init__( self,
                  flywheelSys:Launcher | Agitator,
                  speed:rotations_per_second
                ) -> None:
        # Command Attributes
        self.flywheel_sys:Launcher = flywheelSys
        self.speed:rotations_per_second = speed

        self.setName( f"SetFlywheelSpeed: {flywheelSys.__class__.__name__} @ {speed} rps" )
        self.addRequirements( flywheelSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.flywheel_sys.setDesiredSpeed(
            self.speed
        )

    def end(self, interrupted:bool) -> None:
        self.flywheel_sys.setDesiredSpeed(0.0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False