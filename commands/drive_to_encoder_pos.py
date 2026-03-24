import math

import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.drive_subsystem import DriveSubsystem
from wpimath.controller import PIDController

import ntcore


class DriveToEncoderPos(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, x_target_encoder, y_target_encoder, max_speed, target_threshold) -> None:
        super().__init__()

        self.x_target_encoder = x_target_encoder
        self.y_target_encoder = y_target_encoder

        self.target_angle = 0

        self.max_speed = max_speed
        self.target_threshold = target_threshold

        self.real_distance = 0

        self.drive_sub = drive_sub

        self.addRequirements(self.drive_sub)

        self.x_drive_pid_controller = PIDController(0.2, 0, 0)
        self.y_drive_pid_controller = PIDController(0.2, 0, 0)
        self.rot_pid_controller = PIDController(0.05, 0, 0)

        self.x_drive_pid_controller.setTolerance(target_threshold)
        self.x_drive_pid_controller.setTolerance(target_threshold)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        drive_table = nt_instance.getTable("drive_table")

        self.target_distance_entry = drive_table.getDoubleTopic("target_distance").publish()

    def initialize(self):
        self.drive_sub.reset_encoders()

        self.target_angle = self.drive_sub.get_heading()

    def execute(self) -> None:
        direction_rot = math.atan2(self.y_target_encoder, self.x_target_encoder)

        self.x_drive_pid_controller.setSetpoint(self.x_target_encoder)
        self.y_drive_pid_controller.setSetpoint(self.y_target_encoder)
        self.rot_pid_controller.setSetpoint(self.target_angle)

        x_speed = self.x_drive_pid_controller.calculate(float(abs(self.drive_sub.front_left.get_position().distance_ft)) * math.cos(direction_rot))
        y_speed = self.y_drive_pid_controller.calculate(float(abs(self.drive_sub.front_left.get_position().distance_ft)) * math.sin(direction_rot))
        rot_speed = self.rot_pid_controller.calculate(self.drive_sub.get_heading())

        if self.max_speed > x_speed > -self.max_speed:
            x_speed_limited = x_speed
        elif self.max_speed < x_speed:
            x_speed_limited = self.max_speed
        elif -self.max_speed > x_speed:
            x_speed_limited = -self.max_speed
        else:
            x_speed_limited = 0

        if self.max_speed > y_speed > -self.max_speed:
            y_speed_limited = y_speed
        elif self.max_speed < y_speed:
            y_speed_limited = self.max_speed
        elif -self.max_speed > y_speed:
            y_speed_limited = -self.max_speed
        else:
            y_speed_limited = 0

        self.drive_sub.drive(x_speed_limited, y_speed_limited, rot_speed, False, False)

        self.target_distance_entry.set(self.real_distance)

    def isFinished(self) -> bool:
        return self.x_drive_pid_controller.atSetpoint() and self.y_drive_pid_controller.atSetpoint()

    def end(self, interrupted: bool) -> None:
        self.drive_sub.drive(0, 0, 0, False, False)