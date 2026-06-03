package com.team973.lib.util;

import com.team973.lib.devices.GreyTalonFX.GreyTalonFXConfig;

public class SwerveModuleConfig {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final double angleOffset;

  public final GreyTalonFXConfig driveMotorConfig;
  public final GreyTalonFXConfig angleMotorConfig;

  public SwerveModuleConfig(
      int driveMotorID,
      int angleMotorID,
      int cancoderID,
      double angleOffset,
      GreyTalonFXConfig driveMotorConfig,
      GreyTalonFXConfig angleMotorConfig) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.driveMotorConfig = driveMotorConfig;
    this.angleMotorConfig = angleMotorConfig;
  }
}
