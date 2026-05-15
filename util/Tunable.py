from enum import Enum, auto
import typing as tp

from ntcore.util import ntproperty
ntproperty

from ntcore import NetworkTable, NetworkTableInstance, NetworkTableEntry

'''
reference:
ntproperty definition
https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/LoggedTunableNumber.java
'''

__all__ = ["Tunable"]

class Tunable:

    base_key = "Tuning/"

    ntInst: NetworkTableInstance = NetworkTable.getInstance()

    def __init__(self, key:str, defaultValue:tp.Any, persistent:bool=False, writeDefault:bool=False, updator:tp.Callable[[], None]=lambda:None):
        ## Create nt entry key
        entry_key = ""

        # if the user assigned key starts with "/" start the filepath in the base directory, else put it in "/Tuning"
        if key[0] == "~":
            entry_key = key[1:]
        else:
            entry_key = f"{self.base_key}{key if key[0] != '/' else key[1:]}"

        # create the nt entry object
        self.entry: NetworkTableEntry = self.ntInst.getEntry(entry_key)

        if persistent:
            writeDefault = False
            self.entry.setPersistent()
        else:
            self.entry.clearPersistent()
        
        self.write_default = writeDefault
        self.default_val = defaultValue

        self.reset()

    def reset(self):
        if self.write_default:
            self.entry.setValue(self.default_val)
        else:
            self.entry.setDefaultValue(self.default_val)
        
    def get(self):
        return self.entry.value # same as calling entry.getValue()
    
    def set(self, val:tp.Any):
        self.entry.setValue(val)