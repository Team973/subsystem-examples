package com.team973.lib.util;

import static edu.wpi.first.units.Units.Inches;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModuleSim;
import com.team973.lib.devices.GreyPigeonIO;
import com.team973.lib.devices.GreyPigeonSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class SubsystemManagerSim extends SubsystemManager {
  private final SwerveDriveSimulation m_swerveDriveSimulation;
  private final GreyPigeonIO m_pigeon;

  public SubsystemManagerSim(Logger logger) {
    super(logger);

    DriveTrainSimulationConfig driveTrainSimConfig =
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

    var simStartingPose = new Pose2d(7.18, 5.7, Rotation2d.fromDegrees(180));

    m_swerveDriveSimulation = new SwerveDriveSimulation(driveTrainSimConfig, simStartingPose);

    SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveDriveSimulation);

    m_pigeon =
        new GreyPigeonSim(
            logger.subLogger("pigeon"),
            m_robotInfo.DRIVE_INFO.PIGEON_ID,
            RobotInfo.CANIVORE_CANBUS,
            m_swerveDriveSimulation.getGyroSimulation());
  }

  public GreyPigeonIO getPigeon() {
    return m_pigeon;
  }

  public DriveController initDriveController() {
    Logger driveLogger = getLogger().subLogger("drive");
    RobotInfo.DriveInfo driveInfo = m_robotInfo.DRIVE_INFO;

    return new DriveController(
        driveLogger,
        new SwerveModuleSim(
            0,
            m_swerveDriveSimulation.getModules()[0],
            driveInfo.getFrontLeftConstants(),
            driveLogger.subLogger("swerve/mod0")),
        new SwerveModuleSim(
            1,
            m_swerveDriveSimulation.getModules()[1],
            driveInfo.getFrontRightConstants(),
            driveLogger.subLogger("swerve/mod1")),
        new SwerveModuleSim(
            2,
            m_swerveDriveSimulation.getModules()[2],
            driveInfo.getBackLeftConstants(),
            driveLogger.subLogger("swerve/mod2")),
        new SwerveModuleSim(
            3,
            m_swerveDriveSimulation.getModules()[3],
            driveInfo.getBackRightConstants(),
            driveLogger.subLogger("swerve/mod3")),
        m_pigeon);
  }

  @Override
  public void log() {
    super.log();

    getLogger().log("robotTruthPose", m_swerveDriveSimulation.getSimulatedDriveTrainPose());
  }
}
