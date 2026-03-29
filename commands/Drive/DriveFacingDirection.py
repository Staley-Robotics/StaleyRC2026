import typing

from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d

from wpimath.units import *
from phoenix6.units import *
from phoenix6 import swerve

from ntcore.util import ntproperty

from subsystems import SwerveDrive
from util import RebuiltCalc, FalconLogger

class DriveFacingDirection(Command):
    # Variable Declaration
    drive_rot_kP: percent =  ntproperty("Settings/drive/pid/kP", 2.00, persistent=True)
    drive_rot_kI: percent =  ntproperty("Settings/drive/pid/kI", 0.00, persistent=True)
    drive_rot_kD: percent =  ntproperty("Settings/drive/pid/kD", 0.00, persistent=True)
    
    # Initialization
    def __init__( self,
                  swerveSys: SwerveDrive,
                  getX: typing.Callable[[], float] = lambda: 0.0,
                  getY: typing.Callable[[], float] = lambda: 0.0,
                  getDesiredRot: typing.Callable[[], Rotation2d] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.swerve_sys:SwerveDrive = swerveSys
        self.get_x = getX
        self.get_y = getY
        self.get_desired_rot = getDesiredRot

        # create drive request with constant customizations
        self.drive_req = (
            swerve.requests.FieldCentricFacingAngle()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
                .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self.setName( f"{self.__class__.__name__}" )
        self.addRequirements( swerveSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.swerve_sys.set_control(
            self.drive_req.with_velocity_x( -self.get_y() * self.swerve_sys.max_drive_speed )
                          .with_velocity_y( -self.get_x() * self.swerve_sys.max_rot_speed )

                          .with_deadband( self.swerve_sys.translation_deadband )
                          .with_rotational_deadband( self.swerve_sys.rotation_deadband )

                          .with_target_direction( self.get_desired_rot() )
                          .with_heading_pid( self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kD )
        )
        #Debug:
        FalconLogger.logOutput("/Debug/DriveFacingDir/SetTargetDirection", self.get_desired_rot().degrees())
        FalconLogger.logOutput("/Debug/DriveFacingDir/ReqTargetDirection", self.drive_req.target_direction)
        FalconLogger.logOutput("/Debug/DriveFacingDir/currentRot", self.swerve_sys.get_state().pose.rotation().degrees())
        
    def withRot(self, getDesiredRot:typing.Callable[[], float]) -> typing.Self:
        '''
        modifies getDesiredRot internally and returns self for conciseness
        '''
        self.get_desired_rot = getDesiredRot
        return self

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
