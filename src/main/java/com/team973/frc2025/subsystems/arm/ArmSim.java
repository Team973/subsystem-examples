package com.team973.frc2025.subsystems.arm;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSim extends Arm {
  private final SingleJointedArmSim m_sim;
  private final TalonFXSimState m_motorSimState;

  private double m_lastPoseRad;

  public ArmSim(Logger logger) {
    super(logger);

    m_sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            m_armInfo.MOTOR_GEAR_RATIO,
            0.1956065957,
            0.0,
            Math.toRadians(-60.0),
            Math.toRadians(60.0),
            true,
            Math.toRadians(0.0));

    m_motorSimState = m_motor.getSimState();
    m_lastPoseRad = m_sim.getAngleRads();
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d(0, -0.05, 0.3, new Rotation3d(-m_sim.getAngleRads(), 0, 0));
  }

  @Override
  public void syncSensors() {
    m_motorSimState.addRotorPosition(
        (((m_sim.getAngleRads() - m_lastPoseRad) / (2 * Math.PI)) * m_armInfo.MOTOR_GEAR_RATIO));
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
