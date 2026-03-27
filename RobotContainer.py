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
        self.swerveSys = TunerConstants.create_drivetrain() # TODO: characterization & configs
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        ## Vision
        self.visionSys = Vision( self.swerveSys.add_vision_measurement )
        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 0))
        SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 1))

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

        '''--------------------Create drive commands--------------------'''        
        self.drive_idle = self.swerveSys.apply_request(lambda: swerve.requests.Idle()).ignoringDisable(True).withName('Idling')
        self.drive_brake = self.swerveSys.apply_request(lambda: swerve.requests.SwerveDriveBrake()).withName('Brake')

        self.drive_by_stick = DriveByStick(
            self.swerveSys,
            self.controller1.getLeftX,
            self.controller1.getLeftY,
            self.controller1.getRightX,
        )
        self.drive_facing_target = DriveFacingDirection(
            self.swerveSys,
            self.controller1.getLeftX,
            self.controller1.getLeftY,
            RebuiltCalc.getRotToTarget
        )

        '''--------------------Assign Drive Commands--------------------'''
        ## Defaults
        self.swerveSys.setDefaultCommand(self.drive_by_stick)

        # Idle while the robot is disabled.
        Trigger(DriverStation.isDisabled).whileTrue(self.drive_idle)

        ## Controls
        # Toggle halfspeed
        def toggleHalfSpeed():
            self.swerveSys.drive_max_speed_pct = 0.2 if self.swerveSys.drive_max_speed_pct > 0.5 else 0.6
        self.controller1.leftStick().onTrue(cmd.runOnce(toggleHalfSpeed))

        # Brake (X shape)
        self.controller1.b().toggleOnTrue(self.drive_brake)

        self.controller1.leftBumper().onTrue(cmd.runOnce(self.drive_by_stick.toggleFieldCentric))
        self.controller1.rightBumper().toggleOnTrue(self.drive_facing_target)

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
