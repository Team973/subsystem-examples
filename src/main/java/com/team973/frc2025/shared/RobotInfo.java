package com.team973.frc2025.shared;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Robot info, specs, dimensions. */
public class RobotInfo {
  public static String CANIVORE_CANBUS = "Canivore";
  public String ROBORIO_CANBUS = "";

  public FeatureFlags FEATURE_FLAGS = new FeatureFlags();
  public DriveInfo DRIVE_INFO = new DriveInfo();

  public static class FeatureFlags {
    public boolean ENABLE_DRIVE;
  }

  public static class DriveInfo {
    public int STATUS_SIGNAL_FREQUENCY = 200;

    public int PIGEON_ID = 1;

    public int FRONT_LEFT_MODULE_DRIVE_MOTOR;
    public int FRONT_LEFT_MODULE_STEER_MOTOR;
    public int FRONT_LEFT_MODULE_STEER_ENCODER;
    public double FRONT_LEFT_MODULE_STEER_OFFSET;
    public GreyTalonFX.Config FRONT_LEFT_DRIVE_MOTOR_CONFIG;
    public GreyTalonFX.Config FRONT_LEFT_STEER_MOTOR_CONFIG;

    public int FRONT_RIGHT_MODULE_DRIVE_MOTOR;
    public int FRONT_RIGHT_MODULE_STEER_MOTOR;
    public int FRONT_RIGHT_MODULE_STEER_ENCODER;
    public double FRONT_RIGHT_MODULE_STEER_OFFSET;
    public GreyTalonFX.Config FRONT_RIGHT_DRIVE_MOTOR_CONFIG;
    public GreyTalonFX.Config FRONT_RIGHT_STEER_MOTOR_CONFIG;

    public int BACK_LEFT_MODULE_DRIVE_MOTOR;
    public int BACK_LEFT_MODULE_STEER_MOTOR;
    public int BACK_LEFT_MODULE_STEER_ENCODER;
    public double BACK_LEFT_MODULE_STEER_OFFSET;
    public GreyTalonFX.Config BACK_LEFT_DRIVE_MOTOR_CONFIG;
    public GreyTalonFX.Config BACK_LEFT_STEER_MOTOR_CONFIG;

    public int BACK_RIGHT_MODULE_DRIVE_MOTOR;
    public int BACK_RIGHT_MODULE_STEER_MOTOR;
    public int BACK_RIGHT_MODULE_STEER_ENCODER;
    public double BACK_RIGHT_MODULE_STEER_OFFSET;
    public GreyTalonFX.Config BACK_RIGHT_DRIVE_MOTOR_CONFIG;
    public GreyTalonFX.Config BACK_RIGHT_STEER_MOTOR_CONFIG;

    public double DRIVE_GEAR_RATIO;

    public double ANGLE_GEAR_RATIO;

    public double WHEEL_DIAMETER_METERS;

    public double getWheelCircumferenceMeters() {
      return WHEEL_DIAMETER_METERS * Math.PI;
    }

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public double TRACKWIDTH_METERS;

    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public double WHEELBASE_METERS;

    public double FULL_WIDTH_METERS;

    public double FULL_LENGTH_METERS;

    public double BUMPER_HEIGHT_METERS;

    public double getLinearMetersPerWheelRotations() {
      return WHEEL_DIAMETER_METERS * Math.PI;
    }

    /** Measured Max Speed: 4.724 MPS */
    public double getMaxVelocityMetersPerSecond() {
      return MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
    }

    /** Measured Max Angular Speed: 12.65 RadPS */
    public double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    public double MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

    private SwerveModuleConfig m_frontLeftConstants;

    public SwerveModuleConfig getFrontLeftConstants() {
      if (m_frontLeftConstants == null) {
        m_frontLeftConstants =
            new SwerveModuleConfig(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET,
                FRONT_LEFT_DRIVE_MOTOR_CONFIG,
                FRONT_LEFT_STEER_MOTOR_CONFIG);
      }
      return m_frontLeftConstants;
    }

    private SwerveModuleConfig m_frontRightConstants;

    public SwerveModuleConfig getFrontRightConstants() {
      if (m_frontRightConstants == null) {
        m_frontRightConstants =
            new SwerveModuleConfig(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET,
                FRONT_RIGHT_DRIVE_MOTOR_CONFIG,
                FRONT_RIGHT_STEER_MOTOR_CONFIG);
      }
      return m_frontRightConstants;
    }

    private SwerveModuleConfig m_backLeftConstants;

    public SwerveModuleConfig getBackLeftConstants() {
      if (m_backLeftConstants == null) {
        m_backLeftConstants =
            new SwerveModuleConfig(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET,
                BACK_LEFT_DRIVE_MOTOR_CONFIG,
                BACK_LEFT_STEER_MOTOR_CONFIG);
      }
      return m_backLeftConstants;
    }

    private SwerveModuleConfig m_backRightConstants;

    public SwerveModuleConfig getBackRightConstants() {
      if (m_backRightConstants == null) {
        m_backRightConstants =
            new SwerveModuleConfig(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET,
                BACK_RIGHT_DRIVE_MOTOR_CONFIG,
                BACK_RIGHT_STEER_MOTOR_CONFIG);
      }
      return m_backRightConstants;
    }

    private SwerveDriveKinematics m_swerveKinematics;

    public synchronized SwerveDriveKinematics getSwerveDriveKinematics() {

      if (m_swerveKinematics == null) {
        m_swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
      }
      return m_swerveKinematics;
    }
  }
}
