package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModuleSim;
import com.team973.lib.devices.GreyPigeonIO;
import com.team973.lib.devices.GreyPigeonSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SubsystemManagerSim extends SubsystemManager {
  private final SwerveDriveSimulation m_swerveDriveSimulation;
  private final GreyPigeonIO m_pigeon;
  private final DriveController m_driveController;

  public SubsystemManagerSim(Logger logger) {
    super(logger);

    m_swerveDriveSimulation =
        new SwerveDriveSimulation(
            RobotInfo.DRIVE_INFO.DRIVE_TRAIN_SIMULATION_CONFIG,
            RobotInfo.DRIVE_INFO.SIM_STARTING_POSE);

    SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveDriveSimulation);

    m_pigeon =
        new GreyPigeonSim(
            logger.subLogger("pigeon"),
            RobotInfo.DRIVE_INFO.PIGEON_ID,
            RobotInfo.CANIVORE_CANBUS,
            m_swerveDriveSimulation.getGyroSimulation());

    Logger driveLogger = logger.subLogger("drive", 0.05);

    m_driveController =
        new DriveController(
            logger,
            new SwerveModuleSim(
                0,
                m_swerveDriveSimulation.getModules()[0],
                RobotInfo.DRIVE_INFO.FRONT_LEFT_CONSTANTS,
                driveLogger.subLogger("swerve/mod0")),
            new SwerveModuleSim(
                1,
                m_swerveDriveSimulation.getModules()[1],
                RobotInfo.DRIVE_INFO.FRONT_RIGHT_CONSTANTS,
                driveLogger.subLogger("swerve/mod1")),
            new SwerveModuleSim(
                2,
                m_swerveDriveSimulation.getModules()[2],
                RobotInfo.DRIVE_INFO.BACK_LEFT_CONSTANTS,
                driveLogger.subLogger("swerve/mod2")),
            new SwerveModuleSim(
                3,
                m_swerveDriveSimulation.getModules()[3],
                RobotInfo.DRIVE_INFO.BACK_RIGHT_CONSTANTS,
                driveLogger.subLogger("swerve/mod3")),
            m_pigeon);
  }

  public GreyPigeonIO getPigeon() {
    return m_pigeon;
  }

  public DriveController getDriveController() {
    return m_driveController;
  }

  @Override
  public void log() {
    super.log();

    getLogger().log("robotTruthPose", m_swerveDriveSimulation.getSimulatedDriveTrainPose());
  }
}
