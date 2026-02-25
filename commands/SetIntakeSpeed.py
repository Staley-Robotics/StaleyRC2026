import typing

from commands2 import Command, Subsystem
from subsystems import Intake

class SetIntakeSpeed(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    # Initialization
    def __init__( self,
                  intakeSys:Subsystem,
                  speed:Intake.IntakeSpeeds
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.set_speed:Intake.IntakeSpeeds = speed

        self.setName( f"SetIntakeSpeed: {speed}%" )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        self.intake_sys.setIntakeSpeed(self.set_speed)

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        self.intake_sys.setIntakeSpeed(Intake.IntakeSpeeds.STOP)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False