package com.team973.frc2025.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Arm extends ArmIO {
  protected static final RobotInfo.ArmInfo m_armInfo = RobotInfo.ARM_INFO;

  protected final Logger m_logger;

  protected final GreyTalonFX m_motor;

  private double m_targetPostionDeg;
  private double m_manualInput;

  public Arm(Logger logger) {
    m_logger = logger;
    m_motor =
        new GreyTalonFX(
            m_armInfo.MOTOR_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorRight"));

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Slot0.kS = m_armInfo.ARM_KS;
    motorConfig.Slot0.kV = m_armInfo.ARM_KV;
    motorConfig.Slot0.kA = m_armInfo.ARM_KA;
    motorConfig.Slot0.kP = m_armInfo.ARM_KP;
    motorConfig.Slot0.kI = m_armInfo.ARM_KI;
    motorConfig.Slot0.kD = m_armInfo.ARM_KD;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = m_armInfo.ARM_MOTION_MAGIC_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = m_armInfo.ARM_MOTION_MAGIC_ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = m_armInfo.ARM_MOTION_MAGIC_JERK;

    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        m_armInfo.ARM_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimit = m_armInfo.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_armInfo.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Voltage.PeakForwardVoltage = m_armInfo.PEAK_FORWARD_VOLTAGE;
    motorConfig.Voltage.PeakReverseVoltage = m_armInfo.PEAK_REVERSE_VOLTAGE;

    m_motor.setConfig(motorConfig);
    m_motor.setPosition(0.0);

    m_targetPostionDeg = Preset.One.getPositionDeg();
    m_manualInput = 0.0;
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d(
        new Translation3d(),
        new Rotation3d(
            0, 0, Math.toRadians(motorRotationsToDeg(m_motor.getPosition().getValueAsDouble()))));
  }

  @Override
  public GreyTalonFX getMotor() {
    return m_motor;
  }

  private double degToMotorRotations(double deg) {
    return (deg / 360.0) / m_armInfo.MOTOR_GEAR_RATIO;
  }

  private double motorRotationsToDeg(double motorPostion) {
    return (motorPostion * m_armInfo.MOTOR_GEAR_RATIO) * 360.0;
  }

  @Override
  public void setTargetPreset(Preset preset) {
    m_targetPostionDeg = preset.getPositionDeg();
    setState(State.ClosedLoop);
  }

  @Override
  public void setManualInput(double input) {
    m_manualInput = input;
  }

  @Override
  public double getTargetPositionMotorRot() {
    return degToMotorRotations(m_targetPostionDeg);
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

    m_logger.log(
        "currentPostionDeg", motorRotationsToDeg(m_motor.getPosition().getValueAsDouble()));
    m_logger.log("targetPostionDeg", m_targetPostionDeg);
    m_logger.log("manualInput", m_manualInput);

    m_logger.log("state", getState().toString());
  }

  @Override
  public void reset() {}
}
