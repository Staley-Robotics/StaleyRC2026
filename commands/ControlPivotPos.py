import typing

from commands2 import Command, Subsystem
from subsystems import Intake

class ControlPivotPos(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    # Initialization
    def __init__( self,
                  intakeSys:Subsystem,
                  posInput:typing.Callable[[], float]=lambda:0.0
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.get_speed = posInput

        self.setName( f"ControlPivotPos" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        pass
        # self.intake_sys.setPivotSetpoint(self.get_speed())

    def execute(self) -> None:
        self.intake_sys.setPivotSetpoint(self.get_speed())

    def end(self, interrupted:bool) -> None:
        self.intake_sys.setIntakeSpeed(Intake.IntakeSpeeds.STOP)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False