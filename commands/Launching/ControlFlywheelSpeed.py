import typing

from commands2 import Command, Subsystem
from wpimath.units import percent

from subsystems import Launcher, Agitator

class ControlFlywheelSpeed(Command):
    # Variable Declaration
    flywheel_sys:Launcher = None
    
    # Initialization
    def __init__( self,
                  flywheelSys:Launcher | Agitator,
                  speedInput:typing.Callable[[], percent]=lambda:0.0
                ) -> None:
        # Command Attributes
        self.flywheel_sys:Launcher = flywheelSys
        self.speedInput:typing.Callable[[], percent] = speedInput

        self.setName( f"ControlFlywheelSpeed: {flywheelSys.__class__.__name__}" )
        self.addRequirements( flywheelSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.flywheel_sys.setDesiredSpeed(
            self.speedInput() * self.flywheel_sys.Constants.kMaxExpectedSpeed
        )

    def end(self, interrupted:bool) -> None:
        self.flywheel_sys.setDesiredSpeed(0.0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False