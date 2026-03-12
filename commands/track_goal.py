import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem
from wpimath.controller import PIDController

import ntcore

from subsystems.vision_subsystem import VisionSubsystem


class TurretToPosition(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, vision_sub: VisionSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position
        self.vision_sub = vision_sub

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub, self.vision_sub)

        self.turret_pid_controller = PIDController(0.05, 0, 0)

        self.close_camera_y = 18.7  # O.G. 19.5
        self.far_camera_y = -6.7  # O.G. -9.2
        self.close_encoder = 0.41  # O.G. 0.41
        self.far_encoder = 0.35  # 3/23 10:00 - 0.342, O.G. 0.348

        self.cam_range = abs(self.close_camera_y - self.far_camera_y)
        self.encoder_range = abs(self.close_encoder - self.far_encoder)
        self.cam_angle_ratio = self.encoder_range / self.cam_range

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        turret_table = nt_instance.getTable("turret_table")

        self.current_relative_output_position_entry = turret_table.getDoubleTopic("cur_rel_angle").publish()
        self.turret_pid_output_entry = turret_table.getDoubleTopic("turret_pid_output").publish()

    def execute(self) -> None:
        # Turret Control Block
        self.turret_pid_controller.setSetpoint(self.target_position)
        turret_output = self.turret_pid_controller.calculate(self.vision_sub.turret_x_sub)
        self.turret_sub.set_turret_speed(turret_output)

        # Hood Control Block


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_turret_speed(0)