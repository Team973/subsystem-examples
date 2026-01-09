package com.team973.frc2025.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSim extends Arm {
  private final SingleJointedArmSim m_sim;
  private final TalonFXSimState m_motorSimState;

  private final Angle initialAngle;
  private Angle previousAngle;

  public ArmSim(Logger logger) {
    super(logger);

    initialAngle = Degrees.of(90);
    previousAngle = initialAngle;

    DCMotor krakenX44 = new DCMotor(18.0, 4.05, 275, 1.4, 7530 * 2.0 * Math.PI / 60.0, 2);

    m_sim =
        new SingleJointedArmSim(
            krakenX44,
            1.0 / m_armInfo.MOTOR_GEAR_RATIO,
            0.2,
            m_armInfo.LENGTH_METERS,
            Math.toRadians(-15.0),
            Math.toRadians(195.0),
            true,
            initialAngle.in(Radians));

    m_motorSimState = m_motor.getSimState();
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d(0, -0.05, 0.3, new Rotation3d(-m_sim.getAngleRads(), 0, 0));
  }

  @Override
  public void log() {
    super.log();

    m_logger.log("simAngleRads", m_sim.getAngleRads());
  }

  @Override
  public void syncSensors() {
    Angle armAngle = Radians.of(m_sim.getAngleRads());
    AngularVelocity armAngularVelocity = RadiansPerSecond.of(m_sim.getVelocityRadPerSec());

    Angle diffAngle = armAngle.minus(previousAngle);
    previousAngle = armAngle;

    Angle rotorDiffAngle = diffAngle.div(m_armInfo.MOTOR_GEAR_RATIO);
    AngularVelocity rotorAngularVelocity = armAngularVelocity.div(m_armInfo.MOTOR_GEAR_RATIO);

    m_motorSimState.addRotorPosition(rotorDiffAngle);
    m_motorSimState.setRotorVelocity(rotorAngularVelocity);
    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_sim.update(0.02);

    super.syncSensors();
  }

  @Override
  public void update() {
    super.update();

    m_sim.setInputVoltage(m_motorSimState.getMotorVoltage());
  }
}
