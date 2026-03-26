import typing

from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d

from phoenix6.units import *
from phoenix6 import swerve

from subsystems import SwerveDrive

class DriveByStick(Command):
    # Variable Declaration
    
    # Initialization
    def __init__( self,
                  swerveSys: SwerveDrive,
                  getX: typing.Callable[[], float] = lambda: 0.0,
                  getY: typing.Callable[[], float] = lambda: 0.0,
                  getRot: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.swerve_sys:SwerveDrive = swerveSys
        self.get_x = getX
        self.get_y = getY
        self.get_rot = getRot

        self.field_centric = True

        # create FC drive request with constant customizations
        self.drive_fc = (
            swerve.requests.FieldCentric()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self.drive_rc = (
            swerve.requests.RobotCentric()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )

        self.setName( f"{self.__class__.__name__}" )
        self.addRequirements( swerveSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        if self.field_centric:
            self.swerve_sys.set_control(
                self.drive_fc.with_velocity_x( -self.get_y() * self.swerve_sys.max_drive_speed )
                             .with_velocity_y( -self.get_x() * self.swerve_sys.max_rot_speed )
                             .with_rotational_rate( -self.get_rot() * self.swerve_sys.max_rot_speed )
                             .with_deadband( self.swerve_sys.translation_deadband )
                             .with_rotational_deadband( self.swerve_sys.rotation_deadband )
            )
        else:
            self.swerve_sys.set_control(
                self.drive_rc.with_velocity_x( -self.get_y() * self.swerve_sys.max_drive_speed )
                             .with_velocity_y( -self.get_x() * self.swerve_sys.max_rot_speed )
                             .with_rotational_rate( -self.get_rot() * self.swerve_sys.max_rot_speed )
                             .with_deadband( self.swerve_sys.translation_deadband )
                             .with_rotational_deadband( self.swerve_sys.rotation_deadband )
            )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False

    def toggleFieldCentric(self) -> None:
        self.field_centric = not self.field_centric
