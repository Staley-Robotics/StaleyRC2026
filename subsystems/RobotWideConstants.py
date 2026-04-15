class Constants:
    

    class Launcher:
        #PID Constraints
        k_P:float=0.7
        k_I:float=0.0
        k_D:float=0.03
        k_S:float=0.22
        k_V:float=0.11

        ClosedLoopRampPeriod:float=0.25

        #Limiting Constraints
        statorLimit:float=100
        statorLimitEnable:bool=True
        supplyLimit:float=30
        supplyLimitEnable:bool=True
        supplyLowerLimit:float=30
        supplyLowerTime:float=1.0

    
    class Agitator:
        #PID Constraints
        k_P:float=0.0
        k_I:float=0.0
        k_D:float=0.0
        k_S:float=0.22
        k_V:float=0.112

        ClosedLoopRampPeriod:float=0

        #Limiting Constraints
        statorLimit:float=100
        statorLimitEnable:bool=True
        supplyLimit:float=30
        supplyLimitEnable:bool=True
        supplyLowerLimit:float=30
        supplyLowerTime:float=1.0

    