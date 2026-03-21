import math

import ntcore
import wpilib
import commands2
from commands2 import WaitCommand
from commands2.cmd import waitSeconds

from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.climber_down import ClimberDown
from commands.climber_up import ClimberUp
from commands.drive_command import DriveCommand
from commands.drive_turn_to_angle import DriveTurnToAngle
from commands.extension_to_position import ExtensionToPosition
from commands.hood_and_turret_to_positions import HoodAndTurretToPositions
from commands.hood_down import HoodDown
from commands.hood_to_positon import HoodToPosition
from commands.hood_up import HoodUp
from commands.hopper_out import HopperOut
from commands.input_drive import InputDrive
from commands.intake_in import IntakeIn
from commands.shooter_off import ShooterOff
from commands.shooter_to_velocity import ShooterToVelocity
from commands.track_goal import TrackGoal
from commands.turret_left import TurretLeft
from commands.turret_right import TurretRight
from commands.turret_to_position import TurretToPosition
from constants.position_constants import PositionConstants
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.climber_subsystem import ClimberSubsystem
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.extension_subsystem import ExtensionSubsystem
from subsystems.hopper_subsystem import HopperSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.turret_subsystem import TurretSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.intake_subsystem import IntakeSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.vision_subsystem = VisionSubsystem()
        self.intake_subsystem = IntakeSubsystem()
        self.shooter_subsystem = ShooterSubsystem()
        self.hopper_subsystem = HopperSubsystem()
        self.turret_subsystem = TurretSubsystem()
        self.climber_subsystem = ClimberSubsystem()
        self.extension_subsystem = ExtensionSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )
        self.turret_subsystem.setDefaultCommand(
            HoodToPosition(self.turret_subsystem, 0.5)
        )
        self.shooter_subsystem.setDefaultCommand(
            ShooterToVelocity(self.shooter_subsystem, 3000)
        )
        # Configure Auto Chooser
        # Configure Auto Chooser
        self.chooser = wpilib.SendableChooser()
        self.do_nothing = "Do Nothing"
        self.shoot_preload = "Shoot Preload"
        self.left_one_sweep = "Left One Sweep + Shoot"
        self.right_one_sweep = "Right One Sweep + Shoot"
        self.middle_depot = "Middle Depot + Shoot"
        self.test_auto = "Test Auto"

        self.left_trajectory = "Left Trajectory"
        self.right_trajectory = "Right Trajectory"

        self.chooser.setDefaultOption("Shoot Preload", self.shoot_preload)
        self.chooser.addOption("Do Nothing", self.do_nothing)
        self.chooser.addOption("Left One Sweep + Shoot", self.left_one_sweep)
        self.chooser.addOption("Right One Sweep + Shoot", self.right_one_sweep)
        self.chooser.addOption("Middle Depot + Shoot", self.middle_depot)
        self.chooser.addOption("Test Auto", self.test_auto)

        self.chooser.addOption("Left Trajectory", self.left_trajectory)
        self.chooser.addOption("Right Trajectory", self.right_trajectory)

        wpilib.SmartDashboard.putData("Auto Chooser", self.chooser)

        # self.drive_subsystem.gyro.reset()
        self.drive_subsystem.reset_odometry(Pose2d(0, 0, Rotation2d.fromDegrees(0)))

    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # -- OPERATOR CONTROL BLOCK --
        # Intake In
        commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
            IntakeIn(self.intake_subsystem)
        )
        # Hopper Out
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            HopperOut(self.hopper_subsystem)
        )
        commands2.button.JoystickButton(self.operator_controller, 3).whileTrue(
            IntakeIn(self.intake_subsystem)
        )
        # Hopper Extension Out
        commands2.button.JoystickButton(self.operator_controller, 7).whileTrue(
            ExtensionToPosition(self.extension_subsystem, 0.324)
        )
        # Hopper Extension In
        commands2.button.JoystickButton(self.operator_controller, 8).whileTrue(
            ExtensionToPosition(self.extension_subsystem, 0.14)
        )
        # Shooter Off
        commands2.button.JoystickButton(self.operator_controller, 9).toggleOnTrue(
            ShooterOff(self.shooter_subsystem)
        )
        # Track Goal
        commands2.button.JoystickButton(self.operator_controller, 11).whileTrue(
            TrackGoal(self.turret_subsystem, self.vision_subsystem)
        )
        # Passing Presets
        commands2.button.JoystickButton(self.operator_controller, 12).whileTrue(
            HoodAndTurretToPositions(self.turret_subsystem, self.vision_subsystem, -14, -180)
        )
        commands2.button.JoystickButton(self.operator_controller, 12).toggleOnTrue(
            ShooterToVelocity(self.shooter_subsystem, 6500)
        )
        # Defense Presets
        commands2.button.JoystickButton(self.operator_controller, 13).whileTrue(
            HoodAndTurretToPositions(self.turret_subsystem, self.vision_subsystem, -14, -180)
        )
        commands2.button.JoystickButton(self.operator_controller, 13).toggleOnTrue(
            ShooterToVelocity(self.shooter_subsystem, 6500)
        )
        commands2.button.JoystickButton(self.operator_controller, 13).onTrue(
            ExtensionToPosition(self.extension_subsystem, 0.14)
        )

        # -- DRIVER CONTROL BLOCK --
        # Hopper Extension Ungelation (e ur e ur)
        commands2.button.JoystickButton(self.driver_controller, 1).whileTrue(
            commands2.SequentialCommandGroup(
                commands2.ParallelDeadlineGroup(
                    WaitCommand(1),
                    ExtensionToPosition(self.extension_subsystem, 0.18),
                ),
            commands2.ParallelDeadlineGroup(
                WaitCommand(1),
                ExtensionToPosition(self.extension_subsystem, 0.324),
                )
            ).repeatedly()
        )
        # Climber Up
        commands2.button.JoystickButton(self.driver_controller, 5).whileTrue(
            ClimberUp(self.climber_subsystem)
        )
        # Climber Down
        commands2.button.JoystickButton(self.driver_controller, 10).whileTrue(
            ClimberDown(self.climber_subsystem)
        )
    def periodic(self):
        self.ntcore.putBoolean("LED_TrackingHub", False)

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.command:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        slow_config = TrajectoryConfig(
            AutoConstants.kSlowSpeedMetersPerSecond,
            AutoConstants.kSlowAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # Middle Auto Trajectory to follow. All units in meters.
        mid_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -1, Rotation2d.fromDegrees(0)),
            config,
        )

        # Left Auto Trajectory to follow. All units in meters.
        test_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            [Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            Pose2d(0.8, 0, Rotation2d.fromDegrees(0)),
            Pose2d(0, 0.8, Rotation2d.fromDegrees(0)),
            Pose2d(-0.8, 0, Rotation2d.fromDegrees(0)),
             Pose2d(0, -0.8, Rotation2d.fromDegrees(0))],
            slow_config,
        )

        # Left Sweep LEAVE Trajectory
        left_leave_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(0, 0, Rotation2d.fromDegrees(90)),
             Pose2d(1.825, 0, Rotation2d.fromDegrees(90)),
             Pose2d(3.26, -0.98, Rotation2d.fromDegrees(90))],
            config,
        )

        left_intake_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(3.26, -0.98, Rotation2d.fromDegrees(90)),
             Pose2d(3.26, -2.74, Rotation2d.fromDegrees(90))],
            slow_config,
        )

        left_return_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(3.26, -2.74, Rotation2d.fromDegrees(90)),
             Pose2d(1.825, 0, Rotation2d.fromDegrees(90)),
             Pose2d(0, 0, Rotation2d.fromDegrees(90)),
             Pose2d(-1.45, 0, Rotation2d.fromDegrees(-135))],
            config,
        )

        left_complete_trajectory = left_leave_trajectory + left_intake_trajectory + left_return_trajectory

        # RIGHT Sweep LEAVE Trajectory
        right_leave_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
             Pose2d(-1.825, 0, Rotation2d.fromDegrees(-90)),
             Pose2d(-3.26, 0.98, Rotation2d.fromDegrees(-90))],
            config,
        )

        right_intake_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(-3.26, 0.98, Rotation2d.fromDegrees(-90)),
             Pose2d(-3.26, 2.74, Rotation2d.fromDegrees(-90))],
            slow_config,
        )

        right_return_trajectory = TrajectoryGenerator.generateTrajectory(
            [Pose2d(-3.26, 2.74, Rotation2d.fromDegrees(-90)),
             Pose2d(-1.825, 0, Rotation2d.fromDegrees(-90)),
             Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
             Pose2d(1.45, 0, Rotation2d.fromDegrees(135))],
            config,
        )

        right_complete_trajectory = right_leave_trajectory + right_intake_trajectory + right_return_trajectory

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        thetaController)

        left_trajectory_command = commands2.SwerveControllerCommand(
            left_complete_trajectory,
            self.drive_subsystem.get_pose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.drive_subsystem.set_module_states,
            (self.drive_subsystem,),
        )

        right_trajectory_command = commands2.SwerveControllerCommand(
            right_complete_trajectory,
            self.drive_subsystem.get_pose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.drive_subsystem.set_module_states,
            (self.drive_subsystem,),
        )

        test_trajectory_command = commands2.SwerveControllerCommand(
            test_trajectory,
            self.drive_subsystem.get_pose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.drive_subsystem.set_module_states,
            (self.drive_subsystem,),
        )

        # Start Auto Logic
        auto_selected = self.chooser.getSelected()
        match auto_selected:
            case self.do_nothing:
                return waitSeconds(1)
            case self.shoot_preload:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.turret_subsystem, self.vision_subsystem),
                        ShooterToVelocity(self.shooter_subsystem, 3000)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(8),
                        TrackGoal(self.turret_subsystem, self.vision_subsystem),
                        ShooterToVelocity(self.shooter_subsystem, 3000),
                        HopperOut(self.hopper_subsystem),
                        IntakeIn(self.intake_subsystem),
                        commands2.SequentialCommandGroup(
                            commands2.ParallelDeadlineGroup(
                                WaitCommand(1),
                                ExtensionToPosition(self.extension_subsystem, 0.14),
                            ),
                            commands2.ParallelDeadlineGroup(
                                WaitCommand(1),
                                ExtensionToPosition(self.extension_subsystem, 0.324),
                            )
                        ).repeatedly(),

                        )
                    )
            case self.left_one_sweep:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1.2),
                        InputDrive(self.drive_subsystem, -0.9, 0, 0)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.4),
                        InputDrive(self.drive_subsystem, 0, 0.9, 0)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1),
                        DriveTurnToAngle(self.drive_subsystem, 90)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(4),
                        InputDrive(self.drive_subsystem, -0.2, 0, 0),
                        IntakeIn(self.intake_subsystem),
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1),
                        DriveTurnToAngle(self.drive_subsystem, -90)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        InputDrive(self.drive_subsystem, -0.5, 0, 0)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        InputDrive(self.drive_subsystem, 0, -0.9, 0)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.turret_subsystem, self.vision_subsystem),
                        ShooterToVelocity(self.shooter_subsystem, 3000)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(8),
                        TrackGoal(self.turret_subsystem, self.vision_subsystem),
                        ShooterToVelocity(self.shooter_subsystem, 3000),
                        HopperOut(self.hopper_subsystem),
                        IntakeIn(self.intake_subsystem),
                        commands2.SequentialCommandGroup(
                            commands2.ParallelDeadlineGroup(
                                WaitCommand(1),
                                ExtensionToPosition(self.extension_subsystem, 0.14),
                            ),
                            commands2.ParallelDeadlineGroup(
                                WaitCommand(1),
                                ExtensionToPosition(self.extension_subsystem, 0.324),
                            )
                        ).repeatedly(),
                        )
                    )
            case self.right_one_sweep:
                return commands2.SequentialCommandGroup(
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(1),
                            InputDrive(self.drive_subsystem, 0, 0.9, 0)
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(0.5),
                            InputDrive(self.drive_subsystem, -0.9, 0, 0)
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(2.5),
                            InputDrive(self.drive_subsystem, -0.2, 0, 0),
                            IntakeIn(self.intake_subsystem),
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(2.5),
                            InputDrive(self.drive_subsystem, 0.4, 0, 0),
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(2),
                            InputDrive(self.drive_subsystem, 0, -0.9, 0),
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(0.6),
                            InputDrive(self.drive_subsystem, -0.5, 0, 0),
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(0.5),
                            DriveTurnToAngle(self.drive_subsystem, 45),
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(2),
                            TrackGoal(self.turret_subsystem, self.vision_subsystem),
                            ShooterToVelocity(self.shooter_subsystem, 3000)
                        ),
                        commands2.ParallelDeadlineGroup(
                            WaitCommand(8),
                            TrackGoal(self.turret_subsystem, self.vision_subsystem),
                            ShooterToVelocity(self.shooter_subsystem, 3000),
                            HopperOut(self.hopper_subsystem),
                            IntakeIn(self.intake_subsystem),
                            commands2.SequentialCommandGroup(
                                commands2.ParallelDeadlineGroup(
                                    WaitCommand(1),
                                    ExtensionToPosition(self.extension_subsystem, 0.14),
                                ),
                                commands2.ParallelDeadlineGroup(
                                    WaitCommand(1),
                                    ExtensionToPosition(self.extension_subsystem, 0.324),
                                )
                            ).repeatedly(),
                        )
                    )
            case self.middle_depot:
                return waitSeconds(1)
            case self.test_auto:
                return test_trajectory_command.andThen(
                        InputDrive(self.drive_subsystem, 0, 0, 0)
                    )
            case self.left_trajectory:
                return left_trajectory_command.andThen(
                        InputDrive(self.drive_subsystem, 0, 0, 0)
                    )
            case self.right_trajectory:
                return right_trajectory_command.andThen(
                    InputDrive(self.drive_subsystem, 0, 0, 0)
                )
            case _:
                return waitSeconds(1)