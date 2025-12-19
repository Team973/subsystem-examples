package com.team973.frc2025.subsystems.elevator;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class ElevatorIO extends Subsystem<ElevatorIO.State> {
  private final StateMap<State> m_stateMap;

  public enum State {
    ClosedLoop,
    Manual,
    Off
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

  public abstract void setTargetPostion(double targetPostionHeightinches);

  public abstract double getTargetPositionMotorRot();
}
