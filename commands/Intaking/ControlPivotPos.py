import typing

from commands2 import Command, Subsystem
from ntcore.util import ntproperty

from subsystems import Intake

class ControlPivotPos(Command):
    # Variable Declaration
    intake_sys:Intake = None

    input_mult = ntproperty("/Settings/ControlIntakePos/input mult", 1/25, persistent=True)
    
    # Initialization
    def __init__( self,
                  intakeSys:Intake,
                  posInput:typing.Callable[[], float]=lambda:0.0
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.get_speed = posInput

        self.setName( self.__class__.__name__ )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.intake_sys.setPivotSetpoint(
            # this is techically unsafe for not including range restriction, but that restriction is applied in the subsystem
            self.intake_sys.getPivotSetpoint() + (self.get_speed() * self.input_mult)
        )

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False