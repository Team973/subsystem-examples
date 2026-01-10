package com.team973.frc2025.subsystems.elevator;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class ElevatorIO extends Subsystem<ElevatorIO.State> {
  private final StateMap<State> m_stateMap;

  protected static final RobotInfo.ElevatorInfo m_elevatorInfo = RobotInfo.ELEVATOR_INFO;

  public enum State {
    ClosedLoop,
    Manual,
    Off
  }

  public enum Preset {
    One(m_elevatorInfo.PRESET_ONE),
    Two(m_elevatorInfo.PRESET_TWO),
    Three(m_elevatorInfo.PRESET_THREE);

    private final double m_heightMeters;

    private Preset(double heightMeters) {
      m_heightMeters = heightMeters;
    }

    public double getHeightMeters() {
      return m_heightMeters;
    }
  }

  @SuppressWarnings("unchecked")
  public ElevatorIO() {
    super(State.Off);

    m_stateMap =
        new StateMap<>(
            State.class,
            new StateMap.Entry<>(State.ClosedLoop, new ElevatorStates.ClosedLoop(this)),
            new StateMap.Entry<>(State.Manual, new ElevatorStates.Manual(this)),
            new StateMap.Entry<>(State.Off, new ElevatorStates.Off(this)));
  }

  public StateMap<State> getStateMap() {
    return m_stateMap;
  }

  public abstract Pose3d getPose();

  public abstract GreyTalonFX getMotor();

  public abstract void setTargetPreset(Preset preset);

  public abstract void setManualInput(double input);

  public abstract double getTargetPositionMotorRot();

  public abstract double getManualInput();
}
