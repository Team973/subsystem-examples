package com.team973.lib.util;

import com.team973.frc2025.Robot;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.devices.GreyPigeonIO;

public abstract class SubsystemManager {
  private final Logger m_logger;

  protected SubsystemManager(Logger logger) {
    m_logger = logger;
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

  public abstract DriveController getDriveController();

  public void log() {}
}
