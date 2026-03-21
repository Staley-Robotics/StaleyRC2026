import typing
import math

from commands2 import Command, Subsystem
from wpimath.units import percent
from phoenix6.units import rotations_per_second

from subsystems import Launcher, Agitator

class SetFlywheelSpeed(Command):
    # Variable Declaration
    
    # Initialization
    def __init__( self,
                  flywheelSys:Launcher | Agitator,
                  speed:rotations_per_second,
                  distance
                ) -> None:
        # Command Attributes
        self.flywheel_sys:Launcher | Agitator = flywheelSys
        self.speed:rotations_per_second = speed
        self.distance = distance

        self.setName( f"SetFlywheelSpeed: {flywheelSys.__class__.__name__} @ {speed} rps" )
        self.addRequirements( flywheelSys )

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        self.speed = (math.sqrt((9.8 * self.distance ** 2)/(2 * math.cos(self.distance*math.tan(60)-1.8288)**2)))/(4*math.pi)

        self.flywheel_sys.setDesiredSpeed(
            self.speed
        )

    def end(self, interrupted:bool) -> None:
        self.flywheel_sys.setDesiredSpeed(0.0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False