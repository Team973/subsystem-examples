package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.GreyPigeonIO;

public class SubsystemManagerReal extends SubsystemManager {
  private final GreyPigeonIO m_pigeon;
  private final DriveController m_driveController;

  public SubsystemManagerReal(Logger logger) {
    super(logger);

    m_pigeon =
        new GreyPigeon(
            logger.subLogger("pigeon"), RobotInfo.DRIVE_INFO.PIGEON_ID, RobotInfo.CANIVORE_CANBUS);

    Logger driveLogger = logger.subLogger("drive", 0.05);

    m_driveController =
        new DriveController(
            logger,
            new SwerveModule(
                0, RobotInfo.DRIVE_INFO.FRONT_LEFT_CONSTANTS, driveLogger.subLogger("swerve/mod0")),
            new SwerveModule(
                1,
                RobotInfo.DRIVE_INFO.FRONT_RIGHT_CONSTANTS,
                driveLogger.subLogger("swerve/mod1")),
            new SwerveModule(
                2, RobotInfo.DRIVE_INFO.BACK_LEFT_CONSTANTS, driveLogger.subLogger("swerve/mod2")),
            new SwerveModule(
                3, RobotInfo.DRIVE_INFO.BACK_RIGHT_CONSTANTS, driveLogger.subLogger("swerve/mod3")),
            m_pigeon);
  }

  public GreyPigeonIO getPigeon() {
    return m_pigeon;
  }

  public DriveController getDriveController() {
    return m_driveController;
  }
}
