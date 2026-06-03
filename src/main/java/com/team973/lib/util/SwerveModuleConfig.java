package com.team973.lib.util;

import com.team973.lib.devices.GreyTalonFX;

public class SwerveModuleConfig {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final double angleOffset;

  public final GreyTalonFX.Config driveMotorConfig;
  public final GreyTalonFX.Config angleMotorConfig;

  public SwerveModuleConfig(
      int driveMotorID,
      int angleMotorID,
      int cancoderID,
      double angleOffset,
      GreyTalonFX.Config driveMotorConfig,
      GreyTalonFX.Config angleMotorConfig) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.driveMotorConfig = driveMotorConfig;
    this.angleMotorConfig = angleMotorConfig;
  }
}
