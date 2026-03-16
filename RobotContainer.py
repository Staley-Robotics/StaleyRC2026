## FRC Imports
from wpilib import SendableChooser, SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from commands2 import Command, cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from ntcore.util import ntproperty

# Hardware Lib Imports
from phoenix6 import swerve

## Local Imports
from commands import *
from subsystems import TunerConstants, ClimberClosedLoop, Intake, Launcher, Agitator, Vision

from util import FalconXboxController, Telemetry, ControlMode, RebuiltCalc

class RobotContainer:
    # Variable Declaration
    __autoChooser:SendableChooser = SendableChooser()

    drive_max_speed_pct: percent =  ntproperty("Settings/drive/max speed %", 0.5, persistent=True)
    drive_max_rot_speed: percent =  ntproperty("Settings/drive/max rot speed (rots/sec)", 0.65, persistent=True)

    drive_rot_kP: percent =  ntproperty("Settings/drive/pid/kP", 0.00, persistent=True)
    drive_rot_kI: percent =  ntproperty("Settings/drive/pid/kI", 0.00, persistent=True)
    drive_rot_kD: percent =  ntproperty("Settings/drive/pid/kD", 0.00, persistent=True)

    def __init__(self) -> None:
        ### Controllers
        self.controller1 = FalconXboxController( 0 )
        self.controller2 = FalconXboxController( 1 )
        self.control_mode = ControlMode.TEST

        ### Subsystems
        ## Intake
        self.intakeSys = Intake( 10, 11, 0, 0.3362426 )

        ## Agitator
        self.agitatorSys = Agitator( 12 )

        ## Launcher
        self.launcherSys = Launcher( 13 )

        ## Climber
        self.climbSys = ClimberOpenLoop( 14 )

        #### Weirdo Subsystems
        ## Drive
        self.swerveSys = TunerConstants.create_drivetrain() # TODO: characterization & configs?
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        ## Vision
        self.visionSys = Vision( self.swerveSys.add_vision_measurement )

        ## Initialize RebuiltCalc
        self.gameCalc = RebuiltCalc(lambda: self.swerveSys.get_state().pose)

        ## Auto TODO: re-implement
        # self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        # SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        ### Logging
        SmartDashboard.putData("Subsystems/Intake", self.intakeSys)
        SmartDashboard.putData("Subsystems/Agitator", self.agitatorSys)
        SmartDashboard.putData("Subsystems/Launcher", self.launcherSys)
        SmartDashboard.putData("Subsystems/Climber", self.climbSys)
        SmartDashboard.putData("Subsystems/Swerve", self.swerveSys)
        SmartDashboard.putData("Subsystems/Vision", self.visionSys)

        ### Configure the button bindings
        self.configureDriveBindings()
        # self.configureDriveCharacterizationBrindings() # not properly setup
        match self.control_mode:
            case ControlMode.COMP:
                self.configureCompBindings()
            case ControlMode.PRACTICE:
                self.configurePracticeBindings()
            case ControlMode.DEMO:
                self.configureDemoBindings()
            case ControlMode.TEST:
                self.configureTestBindings()

    def configureCompBindings(self) -> None:
        """
        configures controls for the robot at competition
        """
        ### Driver 1 (Driver)
        #NOTE: drive bindings handled in configureDriveBindings

        ### Driver 2 (Operator)
    def configurePracticeBindings(self) -> None:
        """
        configures controls for the robot in practice
        """
        ### Driver 1 (Driver)
        #NOTE: drive bindings handled in configureDriveBindings

        ### Driver 2 (Operator)
    def configureDemoBindings(self) -> None:
        """
        configures controls for the robot at demo
        """
        ### Driver 1 (Driver)
        #NOTE: drive bindings handled in configureDriveBindings

        ### Driver 2 (Operator)
    def configureTestBindings(self) -> None:
        """
        configures controls for the robot to test subsystems' functionality
        not for testing final bindings, put those in comp
        """
        #NOTE: drive bindings handled in configureDriveBindings
        ## Climbing
        # self.controller1.y().toggleOnTrue(ControlClimberPos(self.climbSys, self.controller1.getRightUpDown))

        ## Intaking
        # Pivot
        # self.controller1.a().toggleOnTrue(ControlPivotPos(self.climbSys, self.controller1.getRightUpDown))
        # #OR
        self.controller1.povDown().onTrue(PivotToPosition(self.intakeSys, 10 ))
        self.controller1.povLeft().onTrue(PivotToPosition(self.intakeSys, 45 ))
        self.controller1.povUp().onTrue(PivotToPosition(self.intakeSys, 90 ))

        # Bawlz
        self.controller1.x().toggleOnTrue(SetIntakeSpeed(self.intakeSys, Intake.IntakeSpeeds.IN))

        ## Launching
        # self.controller1.rightTrigger().whileTrue(RunLauncherByDist(self.launcherSys))
        # self.controller1.rightBumper().whileTrue(ControlFlywheelSpeed(self.agitatorSys, lambda: 3000))
        self.controller1.a().toggleOnTrue(RunFlyWheelByNT(self.launcherSys))
        self.controller1.b().toggleOnTrue(RunAgitatorByNT(self.agitatorSys))
        # this is a stupid way to do waht its doing:
        # self.controller1.rightTrigger(0.01).whileTrue(ControlFlywheelSpeed(self.launcherSys, self.controller1.getRightTriggerAxis))
        # self.controller1.leftTrigger(0.01).whileTrue(ControlFlywheelSpeed(self.agitatorSys, self.controller1.getLeftTriggerAxis))

        ## Climbing
        self.climbSys.setDefaultCommand(ControlClimberSpeed( self.climbSys,
                                                             self.controller1.y().getAsBoolean,
                                                             self.controller1.rightBumper().getAsBoolean, 
                                                             self.controller1.leftBumper().getAsBoolean))

    def configureDriveBindings(self) -> None:
        """
        Control Setup for SwerveDrive
        """

        '''--------------------Create drive requests--------------------'''
        ## speed configs
        self._max_speed = self.drive_max_speed_pct * TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(self.drive_max_rot_speed)
        
        self.drive_fc = ( # field centric
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1  # Add a 10% deadband
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self.drive_rc = ( # robot centric
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1  # Add a 10% deadband
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self.drive_brake = swerve.requests.SwerveDriveBrake()
        self.drive_idle = swerve.requests.Idle()
        self.drive_point = swerve.requests.PointWheelsAt()
        self.drive_fc_with_rot = swerve.requests.FieldCentricFacingAngle()

        '''--------------------Assign Drive Commands--------------------'''
        ## Default
        # NOTE: x = forward & y = left bc wpilib
        self.swerveSys.setDefaultCommand(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc.with_velocity_x( -self.controller1.getLeftY() * self._max_speed )
                                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed )
                                 .with_rotational_rate( -self.controller1.getTriggers() * self._max_angular_rate )
                )
            ).withName('Drive Field Centric')
        )
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        Trigger(DriverStation.isDisabled).whileTrue(
            self.swerveSys.apply_request(lambda: self.drive_idle).ignoringDisable(True).withName('Idling')
        )

        ## Controls
        # Brake
        self.controller1.b().toggleOnTrue(self.swerveSys.apply_request(lambda: self.drive_brake).withName('Brake'))

        # Drive + auto rotate
        self.controller1.rightBumper().toggleOnTrue(
            self.swerveSys.apply_request(
                lambda: self.drive_fc_with_rot.with_velocity_x( -self.controller1.getLeftY() * self._max_speed ) #Rotation2d(-self.controller1.getLeftY(), -self.controller1.getLeftX())
                                              .with_velocity_y( -self.controller1.getLeftX() * self._max_speed )
                                              .with_target_direction( self.gameCalc.getRotToTarget() )
                                              .with_heading_pid( self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kD )
            ).withName('FC + Auto Rotate')
        )
        
        # Drive Robot-centric
        self.controller1.leftBumper().toggleOnTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_rc.with_velocity_x( -self.controller1.getLeftY() * self._max_speed )
                                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed )
                                 .with_rotational_rate( -self.controller1.getRightX() * self._max_angular_rate )
                )
            ).withName('Drive Robot Centric')
        )
    def configureDriveCharacterizationBrindings(self):
        '''
        Setup controls to run Characterization (aka SystemIdentification) on the swervedrive
        these are meant to get data to configure the drive system
        '''
        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self.controller1.back() & self.controller1.y()).whileTrue(
            self.swerveSys.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.controller1.back() & self.controller1.x()).whileTrue(
            self.swerveSys.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.controller1.start() & self.controller1.y()).whileTrue(
            self.swerveSys.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.controller1.start() & self.controller1.x()).whileTrue(
            self.swerveSys.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self.controller1.leftBumper().onTrue(
            self.swerveSys.runOnce(self.swerveSys.seed_field_centric)
        )

        self.swerveSys.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous

        current version auto-generated by phoenix6
        """
    #     # Simple drive forward auton
    #     idle = swerve.requests.Idle()
    #     return cmd.sequence(
    #         # Reset our field centric heading to match the robot
    #         # facing away from our alliance station wall (0 deg).
    #         self.swerveSys.runOnce(
    #             lambda: self.swerveSys.seed_field_centric(Rotation2d.fromDegrees(0))
    #         ),
    #         # Then slowly drive forward (away from us) for 5 seconds.
    #         self.swerveSys.apply_request(
    #             lambda: (
    #                 self._drive_req.with_velocity_x(0.5)
    #                 .with_velocity_y(0)
    #                 .with_rotational_rate(0)
    #             )
    #         )
    #         .withTimeout(5.0),
    #         # Finally idle for the rest of auton
    #         self.swerveSys.apply_request(lambda: idle)
    #     )
