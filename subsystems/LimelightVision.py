import typing

from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians, seconds

# import limelight

from util import FalconLogger

class FalconLimelight:
    def __init__(self, sysID:str, visionApplier:typing.Callable[[Pose2d, seconds, tuple[float]], None], ll_ip:str|None):
        ## Subsystem setup
        self.sysID = sysID
        self.apply_measurement = visionApplier
        # self.ll = limelight.Limelight(ll_ip)

        ## Networktable setup
        table = NetworkTableInstance.getDefault().getTable(sysID)
        self.poseSub = table.getDoubleArrayTopic('botpose_wpiblue').subscribe([])

        self.rotation_stddev = 1.0
    
    # def set_pipeline(self, pipeline_index:int):
    #     self.ll.pipeline_switch(pipeline_index)

    def array2d_to_botpose(self, data:list[float]) -> Pose2d:
        '''
        converts the array2d object received from the limelight into a Pose2d object
        '''
        return Pose2d( data[0], data[1], degreesToRadians(data[5]) )

    # def log_data(self) -> None:
    #     FalconLogger.logInput(f'/Vision/Camera-{self.sysID}/temp', self.ll.get_temp())

    def update_botpose(self) -> Pose2d | None:
        '''
        adds all new vision data on this camera to SS's odometry class
        :returns Pose2d: returns current pose if there was new data, else None
        '''
        # self.log_data()
        data = self.poseSub.readQueue()

        for pose_data in data:
            if pose_data.value[0] != 0:

                self.last_pose = self.array2d_to_botpose(pose_data.value)

                # TODO: change stddev by number of tags
                # TODO: change stddev by distance from tags

                self.apply_measurement(self.last_pose,
                                       pose_data.time/1000000.0 - (pose_data.value[6]/1000),
                                       [1.,1.,self.rotation_stddev]
                                       )


'''
limelight pose_data reference:
0: field relative x position (meters)
1: field relative y position (meters)
2: 
3: 
4: 
5: field relative rotation (degrees)
6: total latency of most recent measurement (milliseconds)
time: global time when measurement was added to networktables (nanoseconds?)
if this is wrong, go look at https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
'''

class Vision(Subsystem):

    Limelight_IDs = {
        'one':"10.49.59.11",
        'two':"10.49.59.12"
        }

    def __init__(self, applyMeasurement:typing.Callable[[Pose2d, seconds, tuple[float]], None]):
        # # maybe make this work later:
        # try:
        #     lls = limelight.discover_limelights(debug=True)
        #     self.cameras = [Limelight( f'limelight-{i}', applyMeasurement) for i in lls]
        # except Exception as err:
        #     print(f'{err}')
        #     self.cameras = [Limelight( f'limelight-{i}', applyMeasurement) for i in self.Limelight_IDs]

        # lls = limelight.discover_limelights(debug=True)
        # print(lls)
        # FalconLogger.logOutput('debug/lls', str(lls))
        # limelight.Limelight('')

        self.cameras: list[FalconLimelight] = []
        for id, ip in self.Limelight_IDs.items():
            try:
                self.cameras.append(FalconLimelight( f'limelight-{id}', applyMeasurement, ip))
            except Exception as err:
                print(f'err while creating limelight, err: {err}')
        # self.cameras = [FalconLimelight( f'limelight-{id}', applyMeasurement, ip) for id, ip in self.Limelight_IDs]

        self.has_received_data = False
        # self.last_pose = Pose2d()

    def periodic(self):
        outputs = [camera.update_botpose() for camera in self.cameras]

        # if any(outputs):
        #     for pose in outputs:
        #         if pose:
        #             self.last_pose = pose
        #             break

        # output = self.camera.update_botpose()
        # if output:
        #     self.last_pose = output
        # # self.has_received_data = self.camera.update_botpose() or self.has_received_data # works with bool return from update

    # def get_last_pose(self) -> Pose2d:
    #     return self.last_pose
    
    def set_rot_stddev(self, val:float) -> None:
        '''
        change the standard deviation for rotation for each camera controlled by the subsystem
        '''
        for camera in self.cameras:
            camera.rotation_stddev = val
    
    def change_pipelines(self, pipeline:int):
        for camera in self.cameras:
            camera.set_pipeline(pipeline)

