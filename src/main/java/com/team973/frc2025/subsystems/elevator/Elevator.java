package com.team973.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Elevator extends ElevatorIO {
  private final Logger m_logger;

  protected final GreyTalonFX m_motor;

  private double m_targetPostionHeightMeters;
  private double m_manualInput;

  public Elevator(Logger logger) {
    m_logger = logger;
    m_motor =
        new GreyTalonFX(
            m_elevatorInfo.MOTOR_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorRight"));

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Slot0.kS = m_elevatorInfo.ELEVATOR_KS;
    motorConfig.Slot0.kV = m_elevatorInfo.ELEVATOR_KV;
    motorConfig.Slot0.kA = m_elevatorInfo.ELEVATOR_KA;
    motorConfig.Slot0.kP = m_elevatorInfo.ELEVATOR_KP;
    motorConfig.Slot0.kI = m_elevatorInfo.ELEVATOR_KI;
    motorConfig.Slot0.kD = m_elevatorInfo.ELEVATOR_KD;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_elevatorInfo.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        m_elevatorInfo.ELEVATOR_MOTION_MAGIC_ACCELERATION;
    motorConfig.MotionMagic.MotionMagicJerk = m_elevatorInfo.ELEVATOR_MOTION_MAGIC_JERK;

    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        m_elevatorInfo.ELEVATOR_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimit = m_elevatorInfo.STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_elevatorInfo.SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Voltage.PeakForwardVoltage = m_elevatorInfo.PEAK_FORWARD_VOLTAGE;
    motorConfig.Voltage.PeakReverseVoltage = m_elevatorInfo.PEAK_REVERSE_VOLTAGE;

    m_motor.setConfig(motorConfig);
    m_motor.setPosition(0.0);

    m_targetPostionHeightMeters = Preset.One.getHeightMeters();
    m_manualInput = 0.0;
  }

  @Override
  public Pose3d getPose() {
    return new Pose3d(
        0,
        0,
        motorRotationsToHeightMeters(m_motor.getPosition().getValueAsDouble())
            * Conversions.Distance.METERS_PER_INCH,
        new Rotation3d());
  }

  @Override
  public GreyTalonFX getMotor() {
    return m_motor;
  }

  protected double heightMetersToMotorRotations(double postionHeight) {
    return postionHeight / m_elevatorInfo.MOTOR_ROT_TO_HEIGHT_METERS;
  }

  private double motorRotationsToHeightMeters(double motorPostion) {
    return motorPostion * m_elevatorInfo.MOTOR_ROT_TO_HEIGHT_METERS;
  }

  @Override
  public void setTargetPreset(Preset preset) {
    m_targetPostionHeightMeters = preset.getHeightMeters();
    setState(State.ClosedLoop);
  }

  @Override
  public void setManualInput(double input) {
    m_manualInput = input;
  }

  @Override
  public double getTargetPositionMotorRot() {
    return heightMetersToMotorRotations(m_targetPostionHeightMeters);
  }

  @Override
  public double getManualInput() {
    return m_manualInput;
  }

  @Override
  public void syncSensors() {}

  @Override
  public void log() {
    double motorRot = m_motor.getPosition().getValueAsDouble();

    m_motor.log();

    m_logger.log("currentPostionHeightMeters", motorRotationsToHeightMeters(motorRot));
    m_logger.log("targetPostionHeightMeters", m_targetPostionHeightMeters);
    m_logger.log("manualInput", m_manualInput);

    m_logger.log("state", getState().toString());
  }

  @Override
  public void reset() {}
}
