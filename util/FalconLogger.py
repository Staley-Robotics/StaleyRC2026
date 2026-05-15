# Python Imports
from typing import Any

# FRC Imports
from wpilib import RobotController, RobotBase
from ntcore import NetworkTableInstance, NetworkTable, StructPublisher, _setNow

from phoenix6.hardware import TalonFX, CANcoder
from rev import SparkMax

LoggableObject = TalonFX | SparkMax | CANcoder

class LoggedObject:
    def __init__(self, key:str, obj:LoggableObject):
        self.key = key
        self.obj = obj

class FalconLogger:
    __outputBase:str = "Real"
    __tbl:NetworkTable = NetworkTableInstance.getDefault().getTable("/")
    __publishers:dict = {}
    __inputs:dict = {}
    __outputs:dict = {}
    __loggedObjects:list[LoggedObject] = [] # currently only implemented for TalonFX, more should be added in future

    def __init__(self, isReplay:bool = False) -> None:
        if RobotBase.isSimulation():
            if isReplay:
                self.__outputBase = "Replay"
            else:
                self.__outputBase = "Sim"

    def setTime(self) -> None:
        _setNow( RobotController.getFPGATime() )
    
    def __updateLoggedObjects(self) -> None:
        #HEY: if you're implementing a new object type in here, please remember to update the type metedata around this file
        # Objects to add: phoenix6 CANCoder
        for logged_obj in self.__loggedObjects:
            match logged_obj.obj:
                case SparkMax():
                    self.logInput(logged_obj.key + "/set speed - pct", logged_obj.obj.get())
                    self.logInput(logged_obj.key + "/duty cycle output", logged_obj.obj.getAppliedOutput())
                    self.logInput(logged_obj.key + "/converted position - rot", logged_obj.obj.getAbsoluteEncoder().getPosition())
                    self.logInput(logged_obj.key + "/converted velocity - rot/s", logged_obj.obj.getAbsoluteEncoder().getVelocity())
                    self.logInput(logged_obj.key + "/output current - amps", logged_obj.obj.getOutputCurrent())
                    self.logInput(logged_obj.key + "/temp - c", logged_obj.obj.getMotorTemperature())
                case TalonFX():
                    #NOTE: this is likely not the best way to log data from phoenix hardware, but is still used for consistency
                    self.logInput(logged_obj.key + "/rotor velocity - rot/s", logged_obj.obj.get_rotor_velocity().value)
                    self.logInput(logged_obj.key + "/converted velocity - rot/s", logged_obj.obj.get_velocity().value)
                    self.logInput(logged_obj.key + "/rotor position - rot", logged_obj.obj.get_rotor_position().value)
                    self.logInput(logged_obj.key + "/converted position - rot", logged_obj.obj.get_position().value)
                    self.logInput(logged_obj.key + "/stator current - amps", logged_obj.obj.get_stator_current().value)
                    self.logInput(logged_obj.key + "/torque current - amps", logged_obj.obj.get_torque_current().value)
                    self.logInput(logged_obj.key + "/stall current - amps", logged_obj.obj.get_motor_stall_current().value)
                    self.logInput(logged_obj.key + "/supply current - amps", logged_obj.obj.get_supply_current().value)
                    self.logInput(logged_obj.key + "/temp - c", logged_obj.obj.get_device_temp().value)
                case CANcoder():
                    #NOTE: this is likely not the best way to log data from phoenix hardware, but is still used for consistency
                    self.logInput(logged_obj.key + "/absolute position - rots", logged_obj.obj.get_absolute_position().value)
                    self.logInput(logged_obj.key + "/velocity - rps", logged_obj.obj.get_velocity().value)
                    self.logInput(logged_obj.key + "/magnet health", logged_obj.obj.get_magnet_health().value.name)
                    self.logInput(logged_obj.key + "/total position - rots", logged_obj.obj.get_position().value)
                case _:
                    print(f"Unsupported object {logged_obj} added to FalconLogger's loggedInputs")

    def writeLog(self) -> None:
        '''
        Update logged objects, then write all Inputs and Outputs to NetworkTables
        '''
        self.__updateLoggedObjects()
        self.__writeLog( "Logging", self.__inputs )
        self.__writeLog( f"{self.__outputBase}Outputs", self.__outputs )     

    def __writeLog(self, key:str, logData:dict) -> None:
        '''
        Write param logData to NetworkTables
        '''
        # Loop Through Records Currently In the Log Data
        # Commit Logs to Network Tables
        for k, v in logData.items():
            path = f"{key}/{k}"
            match v:
                # Arrays of Standard Types
                case list():
                    match v[0]:
                        case bool():
                            self.__tbl.putBooleanArray( path, v )
                        case str():
                            self.__tbl.putStringArray( path, v )
                        case float() | int():
                            self.__tbl.putNumberArray( path, v )
                        case _:
                            if v[0].WPIStruct != None:
                                if path not in self.__publishers:
                                    self.__publishers.update( {path: self.__tbl.getStructArrayTopic( path, type(v[0]) ).publish() } )
                                pub:StructPublisher = self.__publishers[path]
                                pub.set( v )
                            else:
                                print( f"Other type: {type(v)} => {path}: {v}" )
                # Standard single types
                case bool():
                    self.__tbl.putBoolean( path, v )
                case str():
                    self.__tbl.putString( path, v )
                case float() | int():
                    self.__tbl.putNumber( path, v )
                case _: #
                    if v.WPIStruct != None:
                        if path not in self.__publishers:
                            self.__publishers.update( {path: self.__tbl.getStructTopic( path, type(v) ).publish() } )
                        pub:StructPublisher = self.__publishers[path]
                        pub.set( v )
                    else:
                        print( f"Other type: {type(v)} => {path}: {v}" )
        
        # Clear the Log Data Cache
        logData.clear()

    @classmethod
    def logInput(self, key:str, value:Any) -> None:
        '''
        Add a new input data point to be logged

        Use only for direct inputs from hardware (e.g. limit switch state, motor temp or speed, etc.)
        Any values with calculations should be logged as outputs
        '''
        self.__inputs.update({ key: value })

    @classmethod
    def logOutput(self, key:str, value:Any) -> None:
        '''
        Add a new output data point to be logged

        Use only for outputs from calculations (e.g. desired positions, estimated positions, etc.)
        Any values directly from hardware should be logged as inputs
        '''
        self.__outputs.update( {key: value} )
    
    @classmethod
    def addLoggedObject(self, key:str, value:LoggableObject) -> None:
        """
        Add a new object to have its inputs automatically logged
        This function only needs to be called on an object once
        """
        # I dont feel like deduplicating, but its not super risky, so just fix that later
        if type(value) not in {TalonFX, SparkMax, CANcoder}: raise TypeError(f"Object '{value}' of type '{type(value)}' is not supported as a loggedObject in FalconLogger")
        self.__loggedObjects.append(LoggedObject(key, value))