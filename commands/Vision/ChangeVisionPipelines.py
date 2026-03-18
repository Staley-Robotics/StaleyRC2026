import typing

from commands2 import Command, Subsystem
from subsystems import Vision

class ChangeVisionPipelines(Command):
    # Variable Declaration
    vision_sys:Vision = None
    
    # Initialization
    def __init__( self,
                  visionSys:Vision,
                  pipeline:int
                ) -> None:
        # Command Attributes
        self.vision_sys:Vision = visionSys
        self.pipeline = pipeline

        self.setName( f"ChangeVisionPipelines: {pipeline}" )
        self.addRequirements( visionSys ) # doesnt require so pivot commands function uninterupted

    def initialize(self) -> None:
        self.vision_sys.change_pipelines(self.pipeline)

    def execute(self) -> None:
        pass

    def end(self, interrupted:bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False