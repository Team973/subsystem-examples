package com.team973.frc2025.subsystems.shooter;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class ShooterIO extends Subsystem<ShooterIO.State> {
  private final StateMap<State> m_stateMap;

  protected static final RobotInfo.ShooterInfo m_flywheelInfo = RobotInfo.SHOOTER_INFO;

  public enum State {
    ClosedLoop,
    Manual,
    Off
  }

  public enum Preset {
    One(m_flywheelInfo.PRESET_ONE),
    Two(m_flywheelInfo.PRESET_TWO),
    Three(m_flywheelInfo.PRESET_THREE);

    private final double m_velocityRPS;

    private Preset(double velocityRPS) {
      m_velocityRPS = velocityRPS;
    }

    public double getVelocityRPS() {
      return m_velocityRPS;
    }
  }

  @SuppressWarnings("unchecked")
  public ShooterIO() {
    super(State.Off);

    m_stateMap =
        new StateMap<>(
            State.class,
            new StateMap.Entry<>(State.ClosedLoop, new ShooterStates.ClosedLoop(this)),
            new StateMap.Entry<>(State.Manual, new ShooterStates.Manual(this)),
            new StateMap.Entry<>(State.Off, new ShooterStates.Off(this)));
  }

  public StateMap<State> getStateMap() {
    return m_stateMap;
  }

  public abstract Pose3d getPose();

  public abstract GreyTalonFX getMotor();

  public abstract void setTargetPreset(Preset preset);

  public abstract void setManualInput(double volts);

  public abstract double getTargetVelocityMotorRPS();

  public abstract double getManualInput();
}
