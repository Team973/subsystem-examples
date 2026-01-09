package com.team973.frc2025.subsystems.turret;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class TurretIO extends Subsystem<TurretIO.State> {
  private final StateMap<State> m_stateMap;

  protected static final RobotInfo.TurretInfo m_turretInfo = RobotInfo.TURRET_INFO;

  public enum State {
    ClosedLoop,
    Manual,
    CharacterizationKS,
    Off
  }

  public enum Preset {
    One(m_turretInfo.PRESET_ONE),
    Two(m_turretInfo.PRESET_TWO),
    Three(m_turretInfo.PRESET_THREE);

    private final double m_positionDeg;

    private Preset(double positionDeg) {
      m_positionDeg = positionDeg;
    }

    public double getPositionDeg() {
      return m_positionDeg;
    }
  }

  @SuppressWarnings("unchecked")
  public TurretIO() {
    super(State.Off);

    m_stateMap =
        new StateMap<>(
            State.class,
            new StateMap.Entry<>(State.ClosedLoop, new TurretStates.ClosedLoop(this)),
            new StateMap.Entry<>(State.Manual, new TurretStates.Manual(this)),
            new StateMap.Entry<>(State.CharacterizationKS, new TurretStates.CharacterizationKS(this)),
            new StateMap.Entry<>(State.Off, new TurretStates.Off(this)));
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

  public abstract void ksTestIncrment(double incrment); 
}
