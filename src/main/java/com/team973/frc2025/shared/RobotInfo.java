package com.team973.frc2025.shared;

import static edu.wpi.first.units.Units.Inches;

import com.team973.lib.util.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/** Robot info, specs, dimensions. */
public class RobotInfo {
  public static final String CANIVORE_CANBUS = "Canivore"; // "Canivore";
  public static final String ROBORIO_CANBUS = "";

  public static final DriveInfo DRIVE_INFO = new DriveInfo();
  public static final ElevatorInfo ELEVATOR_INFO = new ElevatorInfo();

  public static class ElevatorInfo {
    public final int MOTOR_ID = 20;

    public final double MOTOR_GEAR_RATIO = 10.0 / 56.0;
    public final double MOTOR_ROT_TO_HEIGHT_METERS = MOTOR_GEAR_RATIO * 5.0 * 36.0 * 100.0;

    public final double ELEVATOR_KS = 0.0;
    public final double ELEVATOR_KV = 0.15;
    public final double ELEVATOR_KA = 0.01;
    public final double ELEVATOR_KP = 4.0;
    public final double ELEVATOR_KI = 0.0;
    public final double ELEVATOR_KD = 0.0;

    public final double ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY = 65.0;
    public final double ELEVATOR_MOTION_MAGIC_ACCELERATION = 390.0;
    public final double ELEVATOR_MOTION_MAGIC_JERK = 2000.0;

    public final double ELEVATOR_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.00;

    public final double STATOR_CURRENT_LIMIT = 60.0;
    public final double SUPPLY_CURRENT_LIMIT = 40.0;

    public final double PEAK_FORWARD_VOLTAGE = 12.0;
    public final double PEAK_REVERSE_VOLTAGE = -12.0;
  }

  public static class DriveInfo {
    public final int STATUS_SIGNAL_FREQUENCY = 200;

    public final int PIGEON_ID = 1;

    public final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public final double FRONT_LEFT_MODULE_STEER_OFFSET = -95.273;

    public final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public final double FRONT_RIGHT_MODULE_STEER_OFFSET = 152.929;

    public final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
    public final double BACK_LEFT_MODULE_STEER_OFFSET = 44.472;

    public final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public final double BACK_RIGHT_MODULE_STEER_OFFSET = 100.458;

    public final double DRIVE_GEAR_RATIO =
        (10.0 / 54.0) * (40.0 / 16.0) * (15.0 / 45.0); // x3:10, 6.48:1

    public final double ANGLE_GEAR_RATIO = (10.0 / 22.0) * (16.0 / 88.0); // 12.1:1

    public final double WHEEL_DIAMETER_METERS = 0.1016;
    public final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public static final double TRACKWIDTH_METERS = 0.5334;

    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public static final double WHEELBASE_METERS = 0.5334;

    public final double OPEN_LOOP_RAMP = 0.0;
    public final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public final double ANGLE_KP = 5.5; // 6.5;
    public final double ANGLE_KI = 0.0;
    public final double ANGLE_KD = 0.0;
    public final double ANGLE_KF = 0.0;
    public final double ANGLE_KV = 0.0;

    /* Drive Motor PID Values */
    public final double DRIVE_KP = 0.38;
    public final double DRIVE_KI = 0.0;
    public final double DRIVE_KD = 0.0;
    public final double DRIVE_KF = 0.12;

    /* Motor Inverts */
    public final boolean DRIVE_MOTOR_INVERT = true;
    public final boolean ANGLE_MOTOR_INVERT = true;

    /* Angle Encoder Invert */
    public final boolean CANCODER_INVERT = false;

    // final public  double FALCON_TRAP_FREE_SPEED = 6380.0;
    public final double KRAKEN_TRAP_FREE_SPEED = 6000.0;
    public final double MAX_ACCELERATION_METERS_PER_SECOND = 3.0; // 4.3;
    public final double LINEAR_METERS_PER_WHEEL_ROTATIONS = WHEEL_DIAMETER_METERS * Math.PI;

    /** Measured Max Speed: 4.724 MPS */
    public final double MAX_VELOCITY_METERS_PER_SECOND =
        (KRAKEN_TRAP_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * LINEAR_METERS_PER_WHEEL_ROTATIONS);

    /** Measured Max Angular Speed: 12.65 RadPS */
    public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.3;

    public final SwerveModuleConfig FRONT_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);
    public final SwerveModuleConfig FRONT_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);
    public final SwerveModuleConfig BACK_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);
    public final SwerveModuleConfig BACK_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    public final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    public final DriveTrainSimulationConfig DRIVE_TRAIN_SIMULATION_CONFIG =
        DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(
                COTS.ofMark4(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotor.getKrakenX60(1), // Steer motor is a Kraken
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    3)) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(26), Inches.of(26))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(34), Inches.of(34));

    public final Pose2d SIM_STARTING_POSE = new Pose2d(7.18, 5.7, Rotation2d.fromDegrees(180));
  }
}
