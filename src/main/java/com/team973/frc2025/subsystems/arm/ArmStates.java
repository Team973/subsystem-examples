package com.team973.frc2025.subsystems.arm;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.SubsystemState;

public class ArmStates {
  private abstract static class ArmState implements SubsystemState {
    protected final ArmIO m_arm;

    private ArmState(ArmIO arm) {
      m_arm = arm;
    }
  }

  public static class ClosedLoop extends ArmState {
    public ClosedLoop(ArmIO arm) {
      super(arm);
    }

    public void init() {}

    public void run() {
      m_arm
          .getMotor()
          .setControl(ControlMode.MotionMagicVoltage, m_arm.getTargetPositionMotorRot());
    }

    public void exit() {}
  }

  public static class Manual extends ArmState {
    public Manual(ArmIO arm) {
      super(arm);
    }

    public void init() {}

    public void run() {
      m_arm
          .getMotor()
          .setControl(
              ControlMode.VoltageOut,
              m_arm.getManualInput() * RobotInfo.ARM_INFO.MANUAL_INPUT_TO_VOLTS);
    }

    public void exit() {}
  }

  public static class Off extends ArmState {
    public Off(ArmIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_arm.getMotor().setControl(ControlMode.DutyCycleOut, 0);
    }

    public void exit() {}
  }
}
