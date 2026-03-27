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

from pathplannerlib.auto import AutoBuilder, NamedCommands

## Local Imports
from commands import *
from subsystems import TunerConstants, ClimberClosedLoop, Intake, Launcher, Agitator, Vision

from util import * #FalconXboxController, Telemetry, ControlMode, RebuiltCalc, RebuiltControlBoard

class RobotContainer:
    # Variable Declaration
    drive_max_speed_pct: percent =  ntproperty("Settings/drive/max speed %", 0.60, persistent=True)
    drive_max_rot_speed: percent =  ntproperty("Settings/drive/max rot speed (rots/sec)", 0.65, persistent=True)

    drive_rot_kP: percent =  ntproperty("Settings/drive/pid/kP", 5.00, persistent=True)
    drive_rot_kI: percent =  ntproperty("Settings/drive/pid/kI", 0.00, persistent=True)
    drive_rot_kD: percent =  ntproperty("Settings/drive/pid/kD", 0.00, persistent=True)

    def __init__(self) -> None:
        ### Controllers
        self.controller1 = FalconXboxController( 0 )
        self.controller2 = FalconXboxController( 1 )
        self.controlBoard = RebuiltControlBoard( 2, 3 )
        self.control_mode = ControlMode.TEST

        ### Subsystems
        ## Intake
        self.intakeSys = Intake( 10, 11, 0, -0.4973)

        ## Agitator
        self.agitatorSys = Agitator( 12 )

        ## Launcher
        self.launcherSys = Launcher( 13 )

        ## Climber
        # self.climbSys = ClimberOpenLoop( 14 )

        #### Weirdo Subsystems
        ## Drive
        self.swerveSys = TunerConstants.create_drivetrain() # TODO: characterization & configs?
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        ## Vision
        self.visionSys = Vision( self.swerveSys.add_vision_measurement )

        ## Initialize RebuiltCalc
        self.gameCalc = RebuiltCalc.getInst()
        self.gameCalc.setGetRobotPose(lambda: self.swerveSys.get_state().pose)

        ## Auto
        self.initNamedCommands()
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("AutoChooser", self.autoChooser)

        ### Logging
        SmartDashboard.putData("Subsystems/Intake", self.intakeSys)
        SmartDashboard.putData("Subsystems/Agitator", self.agitatorSys)
        SmartDashboard.putData("Subsystems/Launcher", self.launcherSys)
        # SmartDashboard.putData("Subsystems/Climber", self.climbSys)
        SmartDashboard.putData("Subsystems/Swerve", self.swerveSys)
        SmartDashboard.putData("Subsystems/Vision", self.visionSys)

        ## Configure the button bindings
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
        #NOTE: drive bindings handled in configureDriveBindings
        ## Climbing
        # self.controller1.y().toggleOnTrue(ControlClimberOpenLoop(self.climbSys, self.controller1.getTriggers))
        # self.controlBoard.extra2().whileTrue(ControlClimberOpenLoop(self.climbSys, lambda: -1 if self.controlBoard.switch2().getAsBoolean() else 1))

        ## Intaking
        # Pivot
        (self.controller1.povDown() | self.controller2.povDown()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        (self.controller1.povLeft() | self.controller2.povLeft()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.BOUNCE_UP))
        (self.controller1.povUp() | self.controller2.povUp()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.STORED))

        # Bawlz
        # allow controller 1 or 2 to toggle on a()
        (self.controller1.a() | self.controller2.a()).whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
        # self.controlBoard.extra1().whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.OUT))

        ## Launching
        # handleLaunch = RunLauncherByDist(self.launcherSys)\
        #                 .alongWith(cmd.select(
        #                     {True:SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED),
        #                      False:SetFlywheelSpeed(self.agitatorSys, 0)},
        #                      self.launcherSys.isAtSpeed
        #                 ))
        # handleLaunch = LaunchBalls(self.launcherSys, self.agitatorSys)
        runLauncher = RunLauncherByDist(self.launcherSys)

        self.controller2.rightBumper().onTrue(cmd.runOnce(runLauncher.change_c(+0.5)))
        self.controller2.leftBumper().onTrue(cmd.runOnce(runLauncher.change_c(-0.5)))
        
        self.launcherSys.setDefaultCommand(LauncherDefault(self.launcherSys))

        self.controller2.x().toggleOnTrue(runLauncher) # will trigger launcher (note: player 1 lost this control)
        (self.controller2.rightTrigger(0.3) | self.controller2.leftTrigger(0.3)).whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED))

        # self.controlBoard.launchLow()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_LOW)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_LOW)))
        # self.controlBoard.launchMed()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_MED)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED)))
        # self.controlBoard.launchMed()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_HIGH)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_HIGH)))
        # NOTE: all these agitator speeds are the same

        self.controlBoard.bigRed().whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.EJECT))

        stopLauncher=cmd.runOnce(lambda: (self.launcherSys.setDesiredSpeed(0), self.agitatorSys.setDesiredSpeed(0)))
        stopLauncher.addRequirements(self.agitatorSys, self.launcherSys)
        self.controlBoard.bigBlue().whileTrue(stopLauncher)

        # self.controlBoard.switch3().whileTrue(RunLauncherByNT(self.launcherSys))
        # self.controlBoard.extra3().whileTrue(RunAgitatorByNT(self.agitatorSys))

        ## Misc
        # self.controlBoard.bigRed().whileTrue( Panic() ) # TODO: implement Panic()
        ## Additional Drive Controls
        self.drive_fc_outpost = swerve.requests.RobotCentricFacingAngle()
        self.drive_fc_bump = swerve.requests.RobotCentricFacingAngle()
        self.drive_fc_tower = swerve.requests.RobotCentricFacingAngle()
        self.controlBoard.outpost().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_outpost
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(-90) )
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Outpost')
        )
        self.controlBoard.bump().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_bump
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(45) ) # TODO: normalize to closest proper mult of 45
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Bump')
        )
        self.controlBoard.tower().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_tower
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(180) ) # TODO: normalize to closest proper mult of 45
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Tower')
        )

        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 0))
        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 1))

        ## Disabling
        disableIntake = cmd.runOnce(self.intakeSys.toggleDisabled)
        disableIntake.addRequirements(self.intakeSys)

        self.controlBoard.extra1().onTrue(disableIntake)

        disableAgitator = cmd.runOnce(self.agitatorSys.toggleDisabled)
        disableAgitator.addRequirements(self.agitatorSys)
        
        self.controlBoard.extra2().onTrue(disableAgitator)
        
        disableLauncher = cmd.runOnce(self.launcherSys.toggleDisabled)
        disableLauncher.addRequirements(self.launcherSys)
        
        self.controlBoard.extra3().onTrue(disableLauncher)

        ## Targeting
        self.controlBoard.relayLeft().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.LEFT)))
        self.controlBoard.relayRight().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.RIGHT)))
        self.controlBoard.relayAuto().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.AUTO)))
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
        # self.controller1.y().toggleOnTrue(ControlClimberOpenLoop(self.climbSys, self.controller1.getTriggers))
        # self.controlBoard.extra2().whileTrue(ControlClimberOpenLoop(self.climbSys, lambda: -1 if self.controlBoard.switch2().getAsBoolean() else 1))

        ## Intaking
        # Pivot
        (self.controller1.povDown() | self.controller2.povDown()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        (self.controller1.povLeft() | self.controller2.povLeft()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.BOUNCE_UP))
        (self.controller1.povUp() | self.controller2.povUp()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.STORED))

        # Bawlz
        # allow controller 1 or 2 to toggle on a()
        (self.controller1.a() | self.controller2.a()).whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
        # self.controlBoard.extra1().whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.OUT))

        ## Launching
        # handleLaunch = RunLauncherByDist(self.launcherSys)\
        #                 .alongWith(cmd.select(
        #                     {True:SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED),
        #                      False:SetFlywheelSpeed(self.agitatorSys, 0)},
        #                      self.launcherSys.isAtSpeed
        #                 ))
        # handleLaunch = LaunchBalls(self.launcherSys, self.agitatorSys)
        runLauncher = RunLauncherByDist(self.launcherSys)

        self.controller2.rightBumper().onTrue(cmd.runOnce(runLauncher.change_c(+0.5)))
        self.controller2.leftBumper().onTrue(cmd.runOnce(runLauncher.change_c(-0.5)))
        
        self.launcherSys.setDefaultCommand(LauncherDefault(self.launcherSys))

        self.controller2.x().toggleOnTrue(runLauncher) # will trigger launcher (note: player 1 lost this control)
        (self.controller2.rightTrigger(0.3) | self.controller2.leftTrigger(0.3)).whileTrue(RunAgitatorByNT(self.agitatorSys))

        # self.controlBoard.launchLow()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_LOW)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_LOW)))
        # self.controlBoard.launchMed()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_MED)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED)))
        # self.controlBoard.launchMed()\
        #     .toggleOnTrue(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.SPEED_HIGH)
        #     .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_HIGH)))
        # NOTE: all these agitator speeds are the same

        self.controlBoard.bigRed().whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.EJECT))

        stopLauncher=cmd.runOnce(lambda: (self.launcherSys.setDesiredSpeed(0), self.agitatorSys.setDesiredSpeed(0)))
        stopLauncher.addRequirements(self.agitatorSys, self.launcherSys)
        self.controlBoard.bigBlue().whileTrue(stopLauncher)

        # self.controlBoard.switch3().whileTrue(RunLauncherByNT(self.launcherSys))
        # self.controlBoard.extra3().whileTrue(RunAgitatorByNT(self.agitatorSys))

        ## Misc
        # self.controlBoard.bigRed().whileTrue( Panic() ) # TODO: implement Panic()
        ## Additional Drive Controls
        self.drive_fc_outpost = swerve.requests.RobotCentricFacingAngle()
        self.drive_fc_bump = swerve.requests.RobotCentricFacingAngle()
        self.drive_fc_tower = swerve.requests.RobotCentricFacingAngle()
        self.controlBoard.outpost().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_outpost
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(-90) )
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Outpost')
        )
        self.controlBoard.bump().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_bump
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(45) ) # TODO: normalize to closest proper mult of 45
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Bump')
        )
        self.controlBoard.tower().onTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc_tower
                        .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                        .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                        .with_target_direction( Rotation2d().fromDegrees(180) ) # TODO: normalize to closest proper mult of 45
                        .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric for Tower')
        )

        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 0))
        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 1))

        ## Disabling
        disableIntake = cmd.runOnce(self.intakeSys.toggleDisabled)
        disableIntake.addRequirements(self.intakeSys)

        self.controlBoard.extra1().onTrue(disableIntake)

        disableAgitator = cmd.runOnce(self.agitatorSys.toggleDisabled)
        disableAgitator.addRequirements(self.agitatorSys)
        
        self.controlBoard.extra2().onTrue(disableAgitator)
        
        disableLauncher = cmd.runOnce(self.launcherSys.toggleDisabled)
        disableLauncher.addRequirements(self.launcherSys)
        
        self.controlBoard.extra3().onTrue(disableLauncher)

        ## Targeting
        self.controlBoard.relayLeft().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.LEFT)))
        self.controlBoard.relayRight().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.RIGHT)))
        self.controlBoard.relayAuto().onTrue(cmd.runOnce(self.gameCalc.setDesiredRelay(RelayTarget.AUTO)))

    def configureDriveBindings(self) -> None:
        """
        Standardized control setup for SwerveDrive
        Uses only controller1, controller2 and operator console controls should be set in each respective configure*Bindings func
        """

        '''--------------------Create drive requests--------------------'''
        ## speed configs
        self.drive_max_speed_pct = 0.60 # always start with "full" speed
        self._max_speed = lambda: self.drive_max_speed_pct * TunerConstants.speed_at_12_volts
        self._translational_deadband = lambda: self._max_speed() * 0.05

        self._max_angular_rate = rotationsToRadians(self.drive_max_rot_speed)
        self._rot_deadband = self._max_angular_rate * 0.05
        
        self.drive_fc = ( # field centric
            swerve.requests.FieldCentric() # deadband in application because can vary
            .with_rotational_deadband(
                self._rot_deadband  # Add a 10% deadband
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self.drive_rc = ( # robot centric
            swerve.requests.RobotCentric()
            .with_rotational_deadband(
                self._rot_deadband  # Add a 10% deadband
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
        # NOTE: x = forward & y = left bc wpilib (maybe)
        self.swerveSys.setDefaultCommand(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_fc.with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                                 .with_rotational_rate( -self.controller1.getRightX() * self._max_angular_rate )
                                 .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Field Centric')
        )
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        Trigger(DriverStation.isDisabled).whileTrue(
            self.swerveSys.apply_request(lambda: self.drive_idle).ignoringDisable(True).withName('Idling')
        )

        ## Controls
        # Toggle halfspeed
        def toggleHalfSpeed():
            self.drive_max_speed_pct = 0.25 if self.drive_max_speed_pct > 0.5 else 0.60
        self.controller1.leftStick().onTrue(cmd.runOnce(toggleHalfSpeed))

        # Brake
        self.controller1.b().toggleOnTrue(self.swerveSys.apply_request(lambda: self.drive_brake).withName('Brake'))

        # Drive + auto rotate
        self.controller1.rightBumper().toggleOnTrue(
            self.swerveSys.apply_request(
                lambda: self.drive_fc_with_rot.with_velocity_x( -self.controller1.getLeftY() * self._max_speed() ) #Rotation2d(-self.controller1.getLeftY(), -self.controller1.getLeftX())
                                              .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                                              .with_target_direction( self.gameCalc.getRotToTarget() )
                                              .with_heading_pid( self.drive_rot_kP, self.drive_rot_kI, self.drive_rot_kD )
                                              .with_deadband(self._translational_deadband())
            ).withName('FC + Auto Rotate')
        )
        
        # Drive Robot-centric
        self.controller1.leftBumper().toggleOnTrue(
            self.swerveSys.apply_request(
                lambda: (
                    self.drive_rc.with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
                                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
                                 .with_rotational_rate( -self.controller1.getRightX() * self._max_angular_rate )
                                 .with_deadband(self._translational_deadband())
                )
            ).withName('Drive Robot Centric')
        )
    def configureDriveCharacterizationBindings(self):
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
        return self.autoChooser.getSelected()
        # # # Phoenix's simple drive forward auton
        # # idle = swerve.requests.Idle()
        # return cmd.sequence(
        #     # Reset our field centric heading to match the robot
        #     # facing away from our alliance station wall (0 deg).
        #     self.swerveSys.runOnce(
        #         lambda: self.swerveSys.seed_field_centric(Rotation2d.fromDegrees(0))
        #     ),
        #     # Then slowly drive forward (away from us) for 5 seconds.
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self._drive_req.with_velocity_x(0.5)
        #             .with_velocity_y(0)
        #             .with_rotational_rate(0)
        #         )
        #     )
        #     .withTimeout(5.0),
        #     # Finally idle for the rest of auton
        #     self.swerveSys.apply_request(lambda: idle)
        # )
    def initNamedCommands(self):
        """
        Initialize Named Commands for PathPlanner
        """
        NamedCommands.registerCommand("LaunchBalls", RunLauncherByDist(self.launcherSys).alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED)))
        NamedCommands.registerCommand("DeployIntake", PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        NamedCommands.registerCommand("RunIntake", SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
