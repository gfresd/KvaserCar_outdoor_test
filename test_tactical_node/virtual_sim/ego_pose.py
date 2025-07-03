
class EgoPose:
    def __init__(self,
                 front_x: float,
                 front_y:float,
                 rear_x:float,
                 rear_y:float,
                 vel_x:float,
                 vel_y:float,
                 ego_max_acc:float) -> None:
        self.front_x:float = front_x
        self.front_y:float = front_y
        self.rear_x:float = rear_x
        self.rear_y:float = rear_y
        self.vel_x:float = vel_x
        self.vel_y:float = vel_y
        self.vel_fw:float = vel_x
        self.acc_fw:float = ego_max_acc