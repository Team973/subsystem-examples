package com.team973.frc2025.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
  private final FlywheelSim m_sim;
  private final TalonFXSimState m_motorSimState;

  private double m_lastVelocityRPM;
  private double m_lastTimeSec;

  public ShooterSim(Logger logger) {
    super(logger);

    m_sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), 0.002, m_shooterInfo.MOTOR_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    m_motorSimState = m_motor.getSimState();
    m_lastVelocityRPM = m_sim.getAngularVelocityRPM();
    m_lastTimeSec = Timer.getFPGATimestamp();
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d();
  }

  private double flywheelRPSToMotorRPS(double flywheelRPS) {
    return flywheelRPS * m_shooterInfo.MOTOR_GEAR_RATIO;
  }

  @Override
  public void syncSensors() {
    m_motorSimState.addRotorPosition(
        flywheelRPSToMotorRPS(((m_sim.getAngularVelocityRPM() + m_lastVelocityRPM) / 2.0) / 60.0)
            * (Timer.getFPGATimestamp() - m_lastTimeSec));

    m_lastVelocityRPM = m_sim.getAngularVelocityRPM();
    m_lastTimeSec = Timer.getFPGATimestamp();

    m_motorSimState.setRotorVelocity(flywheelRPSToMotorRPS(m_sim.getAngularVelocityRPM() / 60.0));

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
