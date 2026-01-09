package com.team973.frc2025.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;

public class Shooter extends ShooterIO {
  protected static final RobotInfo.ShooterInfo m_shooterInfo = RobotInfo.SHOOTER_INFO;

  private final Logger m_logger;

  protected final GreyTalonFX m_motor;

  private double m_targetVelocityRPS;
  private double m_manualInput;

  public Shooter(Logger logger) {
    m_logger = logger;
    m_motor =
        new GreyTalonFX(
            m_shooterInfo.MOTOR_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorRight"));

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Slot0.kS = m_shooterInfo.SHOOTER_KS;
    motorConfig.Slot0.kV = m_shooterInfo.SHOOTER_KV;
    motorConfig.Slot0.kA = m_shooterInfo.SHOOTER_KA;
    motorConfig.Slot0.kP = m_shooterInfo.SHOOTER_KP;
    motorConfig.Slot0.kI = m_shooterInfo.SHOOTER_KI;
    motorConfig.Slot0.kD = m_shooterInfo.SHOOTER_KD;

    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        m_shooterInfo.SHOOTER_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimit = m_shooterInfo.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_shooterInfo.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Voltage.PeakForwardVoltage = m_shooterInfo.PEAK_FORWARD_VOLTAGE;
    motorConfig.Voltage.PeakReverseVoltage = m_shooterInfo.PEAK_REVERSE_VOLTAGE;

    m_motor.setConfig(motorConfig);
    m_motor.setPosition(0.0);

    m_targetVelocityRPS = 0.0;
    m_manualInput = 0.0;
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d();
  }

  @Override
  public GreyTalonFX getMotor() {
    return m_motor;
  }

  @Override
  public void setTargetPreset(Preset preset) {
    m_targetVelocityRPS = preset.getVelocityRPS();
    setState(State.ClosedLoop);
  }

  @Override
  public void setManualInput(double input) {
    m_manualInput = input;
  }

  @Override
  public double getTargetVelocityMotorRPS() {
    return m_targetVelocityRPS;
  }

  @Override
  public double getManualInput() {
    return m_manualInput;
  }

  @Override
  public void syncSensors() {}

  @Override
  public void log() {
    m_motor.log();

    m_logger.log("currentVelocityRPS", m_motor.getVelocity().getValueAsDouble());
    m_logger.log("targetVelocityRPS", m_targetVelocityRPS);
    m_logger.log("manualInput", m_manualInput);

    m_logger.log("state", getState().toString());
  }

  @Override
  public void reset() {}
}
