import typing

from commands2 import Command, Subsystem
from ntcore.util import ntproperty

from subsystems import ClimberOpenLoop

class ControlClimberSpeed(Command):
    # Variable Declaration
    climber_sys:ClimberOpenLoop = None

    speed_increment = ntproperty("/Settings/ControlClimberSpeed/speed control increment", 0.1, persistent=True)
    
    # Initialization
    def __init__( self,
                  climberSys:ClimberOpenLoop,
                  runClimberInput:typing.Callable[[], bool]=lambda:False,
                  speedUpInput:typing.Callable[[], bool]=lambda:False,
                  speedDownInput:typing.Callable[[], bool]=lambda:False,
                ) -> None:
        '''
        :param runClimberInput: should be in range [-1,1]
        '''
        # Command Attributes
        self.climber_sys:ClimberOpenLoop = climberSys
        self.runClimberInput = runClimberInput
        self.speedUpInput = speedUpInput
        self.speedDownInput = speedDownInput

        self.speed = 0.0

        self.setName( f"ControlClimberSpeed" )
        self.addRequirements( climberSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        if self.speedUpInput():
            self.speed += self.speed_increment
        if self.speedDownInput():
            self.speed -= self.speed_increment
        if self.runClimberInput():
            self.climber_sys.setSpeed(self.speed)
        else:
            self.climber_sys.setSpeed(0)

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False