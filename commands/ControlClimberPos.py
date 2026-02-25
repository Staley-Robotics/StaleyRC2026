import typing

from commands2 import Command, Subsystem
from ntcore.util import ntproperty

from subsystems import Climber

class ControlClimberPos(Command):
    # Variable Declaration
    climber_sys:Climber = None

    input_mult = ntproperty("/Settings/ControlClimberPos/input mult", 1.0, persistent=True)
    
    # Initialization
    def __init__( self,
                  climberSys:Subsystem,
                  stickInput:typing.Callable[[], float]=lambda:0.0
                ) -> None:
        # Command Attributes
        self.intake_sys:Climber = climberSys
        self.stick_input = stickInput

        self.setName( f"ControlClimberPos" )
        self.addRequirements( climberSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.climber_sys.setDesiredPosition(
            # this is techically unsafe for not including range restriction, but that restriction is applied in the subsystem
            (self.climber_sys.getDesiredPosition() + (self.stick_input() * self.input_mult))
            )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False