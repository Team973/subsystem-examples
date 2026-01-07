package com.team973.frc2025.subsystems.shooter;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.SubsystemState;

public class ShooterStates {
  private abstract static class ShooterState implements SubsystemState {
    protected final ShooterIO m_flywheel;

    private ShooterState(ShooterIO turret) {
      m_flywheel = turret;
    }
  }

  public static class ClosedLoop extends ShooterState {
    public ClosedLoop(ShooterIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_flywheel
          .getMotor()
          .setControl(ControlMode.VelocityVoltage, m_flywheel.getTargetVelocityMotorRPS());
    }

    public void exit() {}
  }

  public static class Manual extends ShooterState {
    public Manual(ShooterIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_flywheel
          .getMotor()
          .setControl(
              ControlMode.VoltageOut,
              m_flywheel.getManualInput() * RobotInfo.SHOOTER_INFO.MANUAL_INPUT_TO_VOLTS);
    }

    public void exit() {}
  }

  public static class Off extends ShooterState {
    public Off(ShooterIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_flywheel.getMotor().setControl(ControlMode.DutyCycleOut, 0);
    }

    public void exit() {}
  }
}
