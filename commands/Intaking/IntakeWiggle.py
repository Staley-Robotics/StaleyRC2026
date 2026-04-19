import typing

from commands2 import Command, Subsystem
from wpimath.units import degrees
from ntcore.util import ntproperty
from subsystems import Intake, IntakeConstants
from util.FalconLogger import FalconLogger


class IntakeWiggle(Command):
    # Variable Declaration
    intake_sys:Intake = None
    
    def __init__( self,
                  intakeSys:Intake,
                  bottomPos:Intake.Positions|degrees,
                  topPos:Intake.Positions|degrees
                ) -> None:
        self.intake_sys:Intake = intakeSys
        self.bottomPos = bottomPos
        self.topPos = topPos

        self.store_pos = self.intake_sys.getPivotSetpoint()

        self.setName( self.__class__.__name__ )
        self.addRequirements( intakeSys )

    def initialize(self) -> None:
        self.intake_sys.setPivotSetpoint(self.topPos)
        self.lastPos = self.intake_sys.getPivotSetpoint()

        self.intake_sys.setIntakeSpeed(Intake.Speeds.IN)
        
    def execute(self) -> None:
        if self.intake_sys.getAtSetpoint(ovverrideTolerance=IntakeConstants.wiggle_tolerance):
            if self.lastPos == self.topPos:
                self.intake_sys.setPivotSetpoint(self.bottomPos)
                self.lastPos = self.bottomPos
            elif self.lastPos == self.bottomPos:
                self.intake_sys.setPivotSetpoint(self.topPos)
                self.lastPos = self.topPos

        FalconLogger.logOutput("/Intake/Outputs/lastPos", self.lastPos)
        FalconLogger.logOutput("/Intake/Outputs/Wiggle At Setpoint", self.intake_sys.getAtSetpoint(ovverrideTolerance=IntakeConstants.wiggle_tolerance))

    def end(self, interrupted:bool) -> None:
        self.intake_sys.setPivotSetpoint( self.store_pos ) #self.intake_sys.getPivotPosition())
        self.intake_sys.setIntakeSpeed( Intake.Speeds.STOP )

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False