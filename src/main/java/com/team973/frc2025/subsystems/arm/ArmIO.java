package com.team973.frc2025.subsystems.arm;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class ArmIO extends Subsystem<ArmIO.State> {
  private final StateMap<State> m_stateMap;

  protected static final RobotInfo.ArmInfo m_armInfo = RobotInfo.ARM_INFO;

  public enum State {
    ClosedLoop,
    Manual,
    Off
  }

  public enum Preset {
    One(m_armInfo.PRESET_ONE),
    Two(m_armInfo.PRESET_TWO),
    Three(m_armInfo.PRESET_THREE);

    private final double m_positionDeg;

    private Preset(double positionDeg) {
      m_positionDeg = positionDeg;
    }

    public double getPositionDeg() {
      return m_positionDeg;
    }
  }

  @SuppressWarnings("unchecked")
  public ArmIO() {
    super(State.Off);

    m_stateMap =
        new StateMap<>(
            State.class,
            new StateMap.Entry<>(State.ClosedLoop, new ArmStates.ClosedLoop(this)),
            new StateMap.Entry<>(State.Manual, new ArmStates.Manual(this)),
            new StateMap.Entry<>(State.Off, new ArmStates.Off(this)));
  }

  public StateMap<State> getStateMap() {
    return m_stateMap;
  }

  public abstract Pose3d getPose();

  public abstract GreyTalonFX getMotor();

  public abstract void setTargetPreset(Preset preset);

  public abstract void setManualInput(double volts);

  public abstract double getTargetPositionMotorRot();

  public abstract double getManualInput();
}
