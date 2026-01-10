package com.team973.frc2025.subsystems.elevator;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.SubsystemState;

public class ElevatorStates {
  private abstract static class ElevatorState implements SubsystemState {
    protected final ElevatorIO m_elevator;

    private ElevatorState(ElevatorIO elevator) {
      m_elevator = elevator;
    }
  }

  public static class ClosedLoop extends ElevatorState {
    public ClosedLoop(ElevatorIO elevator) {
      super(elevator);
    }

    public void init() {}

    public void run() {
      m_elevator
          .getMotor()
          .setControl(ControlMode.MotionMagicVoltage, m_elevator.getTargetPositionMotorRot());
    }

    public void exit() {}
  }

  public static class Manual extends ElevatorState {
    public Manual(ElevatorIO elevator) {
      super(elevator);
    }

    public void init() {}

    public void run() {
      m_elevator
          .getMotor()
          .setControl(
              ControlMode.VoltageOut,
              m_elevator.getManualInput() * RobotInfo.ELEVATOR_INFO.MANUAL_INPUT_TO_VOLTS);
    }

    public void exit() {}
  }

  public static class Off extends ElevatorState {
    public Off(ElevatorIO elevator) {
      super(elevator);
    }

    public void init() {}

    public void run() {
      m_elevator.getMotor().setControl(ControlMode.DutyCycleOut, 0, 0);
    }

    public void exit() {}
  }
}
