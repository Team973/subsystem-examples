package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.GreyPigeonIO;

public class SubsystemManagerReal extends SubsystemManager {
  private final GreyPigeonIO m_pigeon;

  public SubsystemManagerReal(Logger logger) {
    super(logger);

    m_pigeon =
        new GreyPigeon(
            logger.subLogger("pigeon"),
            m_robotInfo.DRIVE_INFO.PIGEON_ID,
            RobotInfo.CANIVORE_CANBUS);
  }

  public GreyPigeonIO getPigeon() {
    return m_pigeon;
  }

  public DriveController initDriveController() {
    Logger driveLogger = getLogger().subLogger("drive");
    RobotInfo.DriveInfo driveInfo = m_robotInfo.DRIVE_INFO;

    return new DriveController(
        driveLogger,
        new SwerveModule(
            0, driveInfo.getFrontLeftConstants(), driveLogger.subLogger("swerve/mod0")),
        new SwerveModule(
            1, driveInfo.getFrontRightConstants(), driveLogger.subLogger("swerve/mod1")),
        new SwerveModule(2, driveInfo.getBackLeftConstants(), driveLogger.subLogger("swerve/mod2")),
        new SwerveModule(
            3, driveInfo.getBackRightConstants(), driveLogger.subLogger("swerve/mod3")),
        m_pigeon);
  }
}
