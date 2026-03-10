from commands2 import Subsystem
from wpilib import  SmartDashboard, Mechanism2d, Color8Bit, Color, RobotState
from wpimath.system.plant import DCMotor
from wpilib.simulation import ElevatorSim
from wpimath.units import *
from rev import EncoderConfig, SparkMax, SparkMaxConfig,ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, SparkBase, LimitSwitchConfig, PersistMode, ResetMode
from phoenix6.units import *

class ClimberConstants:
    
    _kP = 0.0#0.15 Proportional/Present
    _kI = 0.0#      Integral/Past
    _kD = 0.0#0.01  Derivative/Future
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0#0.001 # Feed Forward

    _kAtsetPosTolerence:inches = 1.0
    _pulleyRadius:inches = 0.440
    _pulleyDiameter:inches = _pulleyRadius *2
    _gearRatio = 100.0
    _carriageMass:kilograms = 1.0 # Estimated

    _motorRotsPerHeightInches =  1 / _gearRatio * (_pulleyDiameter * math.pi)

class ClimberPositions:
    BOTTOM:inches = .5 # Minimum height
    TOP:inches = 9 # maximum height
    MIDDLE:inches = (BOTTOM + TOP) / 2

class Climber(Subsystem):
    # Variable Declaration


    # Initialization
    def __init__(self, sysId:int) -> None:
        self.climbMotor = SparkMax(2, SparkMax.MotorType.kBrushless)
        # self.position_request = controls.PositionVoltage(0.0)
        self.leadEncoder = self.climbMotor.getEncoder()

        self.leadEncoder.setPosition(ClimberPositions.BOTTOM)
        self.setPos = 0.0
        self.__pidController = self.climbMotor.getClosedLoopController()

        convFactor = ClimberConstants._motorRotsPerHeightInches
        encConfig = EncoderConfig()
        encConfig = encConfig.positionConversionFactor( convFactor ).velocityConversionFactor( convFactor / 60)
        SmartDashboard.putString("RobotStatus", "Initialized")
        SmartDashboard.putNumber("Climber/BOTTOMPosition", ClimberPositions.BOTTOM)
        SmartDashboard.putNumber("Climber/TOPPosition", ClimberPositions.TOP)

        # configuration
        MotorCfg = SparkMaxConfig()
        MotorCfg = MotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        MotorCfg = MotorCfg.inverted( True )


        clCfg = ClosedLoopConfig()
        clCfg = clCfg.pidf(
            ClimberConstants._kP,
            ClimberConstants._kI,
            ClimberConstants._kD,
            ClimberConstants._kFF,
            ClosedLoopSlot.kSlot0
        )
        clCfg = clCfg.positionWrappingEnabled( False )

        convFactor = ClimberConstants._motorRotsPerHeightInches
        encConfig = EncoderConfig()
        encConfig = encConfig.positionConversionFactor( convFactor ).velocityConversionFactor( convFactor / 60)

        # lsConfig = LimitSwitchConfig()
        # # lsConfig = lsConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        # lsConfig = lsConfig.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        # # lsConfig = lsConfig.forwardLimitSwitchEnabled(False)
        # lsConfig = lsConfig.reverseLimitSwitchEnabled(True)

        # Apply Configs
        MotorCfg.apply(clCfg)
        MotorCfg.apply(encConfig)
        # MotorCfg.apply(lsConfig)

        MotorCfg.apply(encConfig)

        self.climbMotor.configure(MotorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        # Closed Loop
        # self.__pidController = PIDController(
        #     ClimberConstants._kP,
        #     ClimberConstants._kI,
        #     ClimberConstants._kD
        # )

        # Mechanism2d
        self.mech = Mechanism2d(3, 3)
        self.root = self.mech.getRoot("climberRoot", 1.5, 0.5)
        self.ligament = self.root.appendLigament("Climber Arm", 2, 90, 6, Color8Bit(Color.kGreen))
        # Simulation
        self.elevatorSim = ElevatorSim( 
            DCMotor.NEO(1), #gearbox
            ClimberConstants._gearRatio, #gearing / gear ratios
            ClimberConstants._carriageMass, # Carriage Mass/Weight lifted
            inchesToMeters(ClimberConstants._pulleyRadius), # drumRadius/ The radius of the drum that your cable is wrapped around
            inchesToMeters(ClimberPositions.BOTTOM), # MinHeight
            inchesToMeters(ClimberPositions.TOP), # Max Height
            simulateGravity=False, #Gravity
            startingHeight=inchesToMeters(ClimberPositions.BOTTOM), #Starting Height What to put in parameter?
            measurementStdDevs=[0.01, 0.00] # Tolerance????
        )
        self.elevatorSim.setState(ClimberPositions.TOP, (0.0 ))
        # self.setPos = 0
        # self.slot0_configs = configs.Slot0Configs()
        # self.slot0_configs.k_p = ClimberConstants._kPc
        # self.slot0_configs.k_i = ClimberConstants._kI
        # self.slot0_configs.k_d = ClimberConstants._kD
        # self.climbMotor.configurator.apply(self.slot0_configs)

        SmartDashboard.putData("ElevatorSim", self.mech)

        self.__simMotor = SparkMaxSim(self.climbMotor, DCMotor.NEO() )
        self.__simMotor.setPosition( ClimberPositions.BOTTOM )
        

    def run(self) -> None:
        self.__pidController.setReference(
            self.setPos,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        )
        
        current_position = self.climbMotor.get_position().value_as_double()
        error = current_position - self.target_rotations
        SmartDashboard.putNumber("Climber/TargetRot", self.target_rotations)
        SmartDashboard.putNumber("Climber/PositionRot", current_position)
        SmartDashboard.putNumber("Climber/RotError", error)

    def _simulationPeriodic(self):  
        self.motorOutput = self.elevatorSim.getOutput()

    def changeDesiredPos(self, pos:float):
        self.setPos = pos

    def getSetPos(self):
        return self.setPos
    
    def updatesetPos(self):
        self.setPos = self.setPos

    def getsetPosAtSetPos(self):
        return self.setPos == self.setPos

    def getCurPos(self)->inches:
        self.curPos = self.climbMotor.get_position().value

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        self.m_logging.putNumber( "SubsystemData", 0.0 )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        self.m_logging.putNumber( "setPos", self.getsetPos() )
        self.m_logging.putNumber( "Measured", self.m_system )

    # Stop the Subsystem
    def stop(self) -> None:
        pass

    def getAtPosition(self) -> bool:
        return abs(self.getCurPos() - self.setPos )< ClimberConstants._kAtsetPosTolerence