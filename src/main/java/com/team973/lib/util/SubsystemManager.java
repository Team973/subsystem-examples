package com.team973.lib.util;

import com.team973.frc2025.Robot;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.devices.GreyPigeonIO;

public abstract class SubsystemManager {
  private final Logger m_logger;
  protected final RobotInfo m_robotInfo;

  private DriveController m_driveController;

  protected SubsystemManager(Logger logger) {
    m_logger = logger;
    m_robotInfo = new RobotInfo();
  }

  protected Logger getLogger() {
    return m_logger;
  }

  public static SubsystemManager init(Logger logger) {
    if (Robot.isReal()) {
      return new SubsystemManagerReal(logger);
    }

    return new SubsystemManagerSim(logger);
  }

  public abstract GreyPigeonIO getPigeon();

  protected abstract DriveController initDriveController();

  public DriveController getDriveController() {
    if (m_driveController == null) {
      if (m_robotInfo.FEATURE_FLAGS.ENABLE_DRIVE) {
        m_driveController = initDriveController();
      } else {
        m_driveController = null;
      }
    }

    return m_driveController;
  }

  public void log() {}
}
