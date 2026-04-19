import typing

from commands2 import Command, Subsystem
from wpimath.units import percent
from ntcore.util import ntproperty

from subsystems import Agitator

class RunAgitatorByNT(Command):
    '''Made seperate from RunAgitatorByNT because of ntproperty's class-based functionality'''
    # Variable Declaration
    agitator_sys:Agitator = None

    speed = ntproperty("/Settings/RunAgitatorByNT/speed (rps: 0-70)", 0.0, persistent=True)
    
    # Initialization
    def __init__( self,
                  agitatorSys:Agitator,
                ) -> None:
        # Command Attributes
        self.agitator_sys:Agitator = agitatorSys

        self.setName( self.__class__.__name__ )
        self.addRequirements( agitatorSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.agitator_sys.setDesiredSpeed( self.speed )

    def end(self, interrupted:bool) -> None:
        self.agitator_sys.setDesiredSpeed( 0.0 )

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False