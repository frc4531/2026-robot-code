import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class TurretSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.turret_motor = rev.SparkFlex(7, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor = rev.SparkFlex(8, rev.SparkMax.MotorType.kBrushless)

        self.turret_motor.setInverted(True)
        self.hood_motor.setInverted(True)

        self.hood_encoder = self.hood_motor.getEncoder()
        self.turret_encoder = self.turret_motor.getEncoder()

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        hopper_table = nt_instance.getTable("turret_table")

        self.hood_position_entry = hopper_table.getDoubleTopic("hood_position").publish()
        self.turret_position_entry = hopper_table.getDoubleTopic("turret_position").publish()

        self.hood_config = rev.SparkMaxConfig()
        self.turret_config = rev.SparkMaxConfig()

        self.hood_pid_controller = self.hood_motor.getClosedLoopController()
        self.turret_pid_controller = self.hood_motor.getClosedLoopController()

        self.hood_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.setFeedbackSensor(self.hood_motor.))

        self.kP = 22.5
        self.kI = 0
        self.kD = 0
        self.kF = 10 / 6784
        self.min_speed = -0.00005
        self.max_speed = 0.9

        self.left_config.closedLoop.P(self.kP)
        self.left_config.closedLoop.I(self.kI)
        self.left_config.closedLoop.D(self.kD)
        self.left_config.closedLoop.velocityFF(self.kF)
        self.left_config.closedLoop.outputRange(self.min_speed, self.max_speed)

    def periodic(self):
        self.hood_position_entry.set(self.get_hood_position())
        self.turret_position_entry.set(self.get_turret_position())

    def get_hood_position(self):
        return self.hood_encoder.getPosition()

    def get_turret_position(self):
        return self.turret_encoder.getPosition()

    def set_turret_speed(self, speed):
        self.turret_motor.set(speed)

    def set_hood_speed(self, speed):
        self.hood_motor.set(speed)
