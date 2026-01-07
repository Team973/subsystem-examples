package com.team973.frc2025.subsystems.turret;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretSim extends Turret {
  private final SingleJointedArmSim m_sim;
  private final TalonFXSimState m_motorSimState;

  private double m_lastPoseRad;

  public TurretSim(Logger logger) {
    super(logger);

    m_sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            m_turretInfo.MOTOR_GEAR_RATIO,
            0.02,
            0.0,
            Math.toRadians(-165.0),
            Math.toRadians(165.0),
            false,
            Math.toRadians(0.0));

    m_motorSimState = m_motor.getSimState();
    m_lastPoseRad = m_sim.getAngleRads();
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d(new Translation3d(), new Rotation3d(0, 0, m_sim.getAngleRads()));
  }

  @Override
  public void syncSensors() {
    m_motorSimState.addRotorPosition(
        (((m_sim.getAngleRads() - m_lastPoseRad) / (2 * Math.PI)) * m_turretInfo.MOTOR_GEAR_RATIO));
    m_lastPoseRad = m_sim.getAngleRads();

    m_motorSimState.setRotorVelocity(m_sim.getVelocityRadPerSec());

    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    super.syncSensors();
  }

  @Override
  public void update() {
    super.update();

    m_sim.setInputVoltage(m_motorSimState.getMotorVoltage());

    m_sim.update(0.02);
  }
}
