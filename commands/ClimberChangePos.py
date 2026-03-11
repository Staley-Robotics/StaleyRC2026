import typing

from commands2 import Command, Subsystem
from subsystems import Climber, ClimberPositions
from ntcore.util import ntproperty

class ClimberChangePos(Command):
    # Variable Declaration
    climber_Sys:Climber = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    controlSpeed = ntproperty('/Settings/climber/controlmult', 0.08, persistent=True)

    
    # Initialization
    def __init__( self,
                  ClimberSys:Subsystem,
                #   joystick: typing.Callable[[], float] = lambda: 0.0
                  ChangeAmount:float
                ) -> None:
        # Command Attributes
        self.climber_Sys:Climber = ClimberSys
        # self.m_getValue = joystick
        self.setName( "ClimberChangePos" )
        self.addRequirements( ClimberSys )
        self.changeAmount = ChangeAmount
    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        climberPos = (self.climber_Sys.getSetPos() + (self.changeAmount* self.controlSpeed))
        self.climber_Sys.changeDesiredPos(climberPos)

    
    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False