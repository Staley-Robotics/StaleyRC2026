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
        self.control_mode = ControlMode.DEMO

        ### Subsystems
        ## Intake
        self.intakeSys = Intake( 10, 11, 0, 0.182617, lambda: not self.controlBoard.switch2().getAsBoolean())

        ## Agitator
        self.agitatorSys = Agitator( 12, self.controlBoard.switch1().getAsBoolean )

        ## Launcher
        self.launcherSys = Launcher( 13, self.controlBoard.switch1().getAsBoolean )

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
        RebuiltCalc.assignSwerveSys(self.swerveSys)

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
        #NOTE: other drive bindings handled in configureDriveBindings
        self.drive_facing_target = DriveFacingDirection(
            self.swerveSys,
            self.controller1.getLeftX,
            self.controller1.getLeftY,
            RebuiltCalc.getRotToTarget
        )
        self.controller1.rightBumper().toggleOnTrue(self.drive_facing_target)
        ## Intaking
        # Pivot
        (self.controller1.povDown() | self.controller2.povDown()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        (self.controller1.povLeft() | self.controller2.povLeft()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.BOUNCE_UP))
        (self.controller1.povUp() | self.controller2.povUp()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.STORED))
        (self.controller1.povRight() | self.controller2.povRight()).whileTrue(IntakeWiggle(self.intakeSys, bottomPos=Intake.Positions.BOUNCE_DOWN, topPos=Intake.Positions.BOUNCE_UP))

        # Bawlz
        # allow controller 1 or 2 to hold on a(), and c1 to hold either trigger
        ((self.controller1.a() | self.controller2.a()) | (self.controller1.rightTrigger(0.3) | self.controller1.leftTrigger(0.3))).whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
        # Eject
        self.controlBoard.extra1().whileTrue(
            SetIntakeSpeed(self.intakeSys, Intake.Speeds.OUT)
            .alongWith(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.EJECT))
            .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED))
        ) # Poop

        ## Launching
        runLauncher = RunLauncherByDist(self.launcherSys)

        # self.controller2.rightBumper().onTrue(cmd.runOnce(runLauncher.change_c(+0.5)))
        # self.controller2.leftBumper().onTrue(cmd.runOnce(runLauncher.change_c(-0.5)))
        
        self.launcherSys.setDefaultCommand(LauncherDefault(self.launcherSys))

        self.controller2.x().toggleOnTrue(runLauncher)
        self.controller2.leftTrigger(0.3).whileTrue(RunLauncherByDist(self.launcherSys)) # uses different instance of command for better control
        self.controller2.rightTrigger(0.3).whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED).onlyWhile(
            # only launch if launcher at speed OR switch say yes
            lambda: (self.launcherSys.isAtSpeed() and self.launcherSys.getDesiredSpeed() != Launcher.LauncherSpeeds.WAIT) or self.controlBoard.switch1().getAsBoolean()
            )
        )

        self.controlBoard.bigRed().whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.EJECT))

        stopLauncher=cmd.runOnce(lambda: (self.launcherSys.setDesiredSpeed(0), self.agitatorSys.setDesiredSpeed(0)))
        stopLauncher.addRequirements(self.agitatorSys, self.launcherSys)
        self.controlBoard.bigBlue().whileTrue(stopLauncher)

        # self.controlBoard.switch3().whileTrue(RunLauncherByNT(self.launcherSys))
        # self.controlBoard.extra3().whileTrue(RunAgitatorByNT(self.agitatorSys))

        ## Misc
        # self.controlBoard.bigRed().whileTrue( Panic() ) # TODO: implement Panic()
        ## Additional Drive Controls
        # self.drive_fc_outpost = swerve.requests.RobotCentricFacingAngle()
        # self.drive_fc_bump = swerve.requests.RobotCentricFacingAngle()
        # self.drive_fc_tower = swerve.requests.RobotCentricFacingAngle()
        # self.controlBoard.outpost().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_outpost
        #                 .with_velocity_x( -self.controller1.getLeftY() * self.swerveSys.max_drive_speed )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self.swerveSys.max_drive_speed )
        #                 .with_target_direction( Rotation2d().fromDegrees(-90) )
        #                 .with_deadband(self.swerveSys.translation_deadband)
        #         )
        #     ).withName('Drive Field Centric for Outpost')
        # )
        # self.controlBoard.bump().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_bump
        #                 .with_velocity_x( -self.controller1.getLeftY() * self.swerveSys.max_drive_speed )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self.swerveSys.max_drive_speed )
        #                 .with_target_direction( Rotation2d().fromDegrees(45) ) # TODO: normalize to closest proper mult of 45
        #                 .with_deadband(self.swerveSys.translation_deadband)
        #         )
        #     ).withName('Drive Field Centric for Bump')
        # )
        # self.controlBoard.tower().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_tower
        #                 .with_velocity_x( -self.controller1.getLeftY() * self.swerveSys.max_drive_speed )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self.swerveSys.max_drive_speed )
        #                 .with_target_direction( Rotation2d().fromDegrees(180) ) # TODO: normalize to closest proper mult of 45
        #                 .with_deadband(self.swerveSys.translation_deadband)
        #         )
        #     ).withName('Drive Field Centric for Tower')
        # )

        # SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 0))
        # SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 1))

        ## Disabling
        # disableIntake = cmd.runOnce(self.intakeSys.toggleDisabled)
        # disableIntake.addRequirements(self.intakeSys)

        # self.controlBoard.extra1().onTrue(disableIntake)

        # disableAgitator = cmd.runOnce(self.agitatorSys.toggleDisabled)
        # disableAgitator.addRequirements(self.agitatorSys)
        
        # self.controlBoard.extra2().onTrue(disableAgitator)
        
        # disableLauncher = cmd.runOnce(self.launcherSys.toggleDisabled)
        # disableLauncher.addRequirements(self.launcherSys)
        
        # self.controlBoard.extra3().onTrue(disableLauncher)

        ## Targeting
        self.controlBoard.relayLeft().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.LEFT)))
        self.controlBoard.relayRight().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.RIGHT)))
        self.controlBoard.relayAuto().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.AUTO)))

        self.controlBoard.switch3().onFalse(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.DONT)))
        self.controlBoard.switch3().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.AUTO)))

        self.controller1.y().onTrue(cmd.runOnce(RebuiltCalc.toggleUseRelayTargeting))
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

        Uses only 1 controller and the control board
        simplified controls to be more easily handled by untrained drivers
        """
        '''------------------------Controller------------------------'''
        ## Intaking
        # Pivot
        (self.controller1.povDown() | self.controller2.povDown()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        (self.controller1.povLeft() | self.controller2.povLeft()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.BOUNCE_UP))
        (self.controller1.povUp() | self.controller2.povUp()).onTrue(PivotToPosition(self.intakeSys, Intake.Positions.STORED))
        (self.controller1.povRight() | self.controller2.povRight()).whileTrue(IntakeWiggle(self.intakeSys, bottomPos=Intake.Positions.BOUNCE_DOWN, topPos=Intake.Positions.BOUNCE_UP))

        # Bawlz
        # allow controller 1 to hold on a(), and c1 to hold either trigger
        (self.controller1.a()).whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
        # Eject
        self.controlBoard.extra1().whileTrue(
            SetIntakeSpeed(self.intakeSys, Intake.Speeds.OUT)
            .alongWith(SetFlywheelSpeed(self.launcherSys, Launcher.LauncherSpeeds.EJECT))
            .alongWith(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED))
        ) # Poop

        ## Launching
        runLauncher = RunLauncherByNT(self.launcherSys)

        self.controller1.rightBumper().onTrue(cmd.runOnce(runLauncher.changeSpeed(+0.5)))
        self.controller1.leftBumper().onTrue(cmd.runOnce(runLauncher.changeSpeed(-0.5)))
        
        self.controller1.leftTrigger(0.3).whileTrue(runLauncher)
        self.controller1.rightTrigger(0.3).whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED))

        self.controlBoard.bigRed().whileTrue(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.EJECT))

        stopLauncher=cmd.runOnce(lambda: (self.launcherSys.setDesiredSpeed(0), self.agitatorSys.setDesiredSpeed(0)))
        stopLauncher.addRequirements(self.agitatorSys, self.launcherSys)
        self.controlBoard.bigBlue().whileTrue(stopLauncher)

        '''------------------------Control Board------------------------'''

        self.controlBoard.extra1().whileTrue(runLauncher)
        self.controlBoard.extra2().whileTrue(RunAgitatorByNT(self.agitatorSys))

        endSwerveCommand = cmd.runOnce(lambda: None).withName('This shouldnt be here more than a frame')
        endSwerveCommand.addRequirements(self.swerveSys)

        self.controlBoard.switch2().onChange(
            cmd.runOnce(self.intakeSys.toggleDisabled)
        )

        # self.controlBoard.switch3().onTrue(
        #     cmd.runOnce(self.swerveSys.setDefaultCommand(self.swerveSys.apply_request(lambda: swerve.requests.Idle()).ignoringDisable(True).withName('Disabled')))
        #     .andThen(self.swerveSys.apply_request(lambda: swerve.requests.Idle()).ignoringDisable(True).withName('Disabled'))
        # )
        # self.controlBoard.switch3().onFalse(
        #     cmd.runOnce(self.swerveSys.setDefaultCommand(self.drive_by_stick))
        #     .andThen(endSwerveCommand)
        # )
        
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
        (self.controller1.povRight() | self.controller2.povRight()).whileTrue(IntakeWiggle(self.intakeSys, bottomPos=Intake.Positions.BOUNCE_DOWN, topPos=Intake.Positions.BOUNCE_UP))

        # Bawlz
        # allow controller 1 or 2 to toggle on a()
        # (self.controller1.a() | self.controller2.a()).whileTrue(SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
        (self.controller1.a() | self.controller2.a()).whileTrue(RunIntakeByNT(self.intakeSys))
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
        # self.drive_fc_outpost = swerve.requests.RobotCentricFacingAngle()
        # self.drive_fc_bump = swerve.requests.RobotCentricFacingAngle()
        # self.drive_fc_tower = swerve.requests.RobotCentricFacingAngle()
        # self.controlBoard.outpost().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_outpost
        #                 .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
        #                 .with_target_direction( Rotation2d().fromDegrees(-90) )
        #                 .with_deadband(self._translational_deadband())
        #         )
        #     ).withName('Drive Field Centric for Outpost')
        # )
        # self.controlBoard.bump().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_bump
        #                 .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
        #                 .with_target_direction( Rotation2d().fromDegrees(45) ) # TODO: normalize to closest proper mult of 45
        #                 .with_deadband(self._translational_deadband())
        #         )
        #     ).withName('Drive Field Centric for Bump')
        # )
        # self.controlBoard.tower().onTrue(
        #     self.swerveSys.apply_request(
        #         lambda: (
        #             self.drive_fc_tower
        #                 .with_velocity_x( -self.controller1.getLeftY() * self._max_speed() )
        #                 .with_velocity_y( -self.controller1.getLeftX() * self._max_speed() )
        #                 .with_target_direction( Rotation2d().fromDegrees(180) ) # TODO: normalize to closest proper mult of 45
        #                 .with_deadband(self._translational_deadband())
        #         )
        #     ).withName('Drive Field Centric for Tower')
        # )

        # SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 0))
        # SmartDashboard.putData(ChangeVisionPipelines(self.visionSys, 1))

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
        self.controlBoard.relayLeft().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.LEFT)))
        self.controlBoard.relayRight().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.RIGHT)))
        self.controlBoard.relayAuto().onTrue(cmd.runOnce(lambda: RebuiltCalc.setDesiredRelay(RelayTarget.AUTO)))

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
            self.controlBoard.switch3().getAsBoolean
        )

        '''--------------------Assign Drive Commands--------------------'''
        ## Defaults
        self.swerveSys.setDefaultCommand(self.drive_by_stick)

        # Idle while the robot is disabled.
        Trigger(DriverStation.isDisabled).whileTrue(self.drive_idle)

        ## Controls
        # Toggle halfspeed
        def toggleHalfSpeed():
            self.swerveSys.drive_max_speed_pct = 0.3 if self.swerveSys.drive_max_speed_pct > 0.5 else 0.6
        self.controller1.leftStick().onTrue(cmd.runOnce(toggleHalfSpeed))

        # Brake (X shape)
        self.controller1.b().toggleOnTrue(self.drive_brake)

        self.controller1.leftBumper().onTrue(cmd.runOnce(self.drive_by_stick.toggleFieldCentric))

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

    def initNamedCommands(self):
        """
        Initialize Named Commands for PathPlanner
        """
        NamedCommands.registerCommand("LaunchBalls", RunLauncherByDist(self.launcherSys).alongWith(
                cmd.waitUntil(lambda: self.launcherSys.isAtSpeed() and self.launcherSys.getDesiredSpeed() > Launcher.LauncherSpeeds.WAIT)
                .andThen(SetFlywheelSpeed(self.agitatorSys, Agitator.Speeds.SPEED_MED)
                         .alongWith(IntakeWiggle(self.intakeSys, bottomPos=Intake.Positions.BOUNCE_DOWN, topPos=Intake.Positions.BOUNCE_UP))))
            )
        NamedCommands.registerCommand("DeployIntake", PivotToPosition(self.intakeSys, Intake.Positions.INTAKING))
        NamedCommands.registerCommand("RunIntake", SetIntakeSpeed(self.intakeSys, Intake.Speeds.IN))
