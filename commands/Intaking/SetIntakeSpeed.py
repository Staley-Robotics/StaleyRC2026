import typing

from commands2 import Command, Subsystem
from subsystems import Intake, percent

class SetIntakeSpeed(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    # Initialization
    def __init__( self,
                  intakeSys:Intake,
                  speed:Intake.Speeds|percent
                ) -> None:
        # Command Attributes
        self.intake_sys:Intake = intakeSys
        self.set_speed:Intake.Speeds|percent = speed

        self.setName( f"SetIntakeSpeed: {speed}%" )
        # self.addRequirements( intakeSys ) # doesnt require subsys so pivot commands can function uninterupted

    def initialize(self) -> None:
        self.intake_sys.setIntakeSpeed(self.set_speed)

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        self.intake_sys.setIntakeSpeed(Intake.Speeds.STOP)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False