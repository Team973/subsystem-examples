package com.team973.frc2025.subsystems.turret;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.SubsystemState;
import edu.wpi.first.wpilibj.RobotController;

public class TurretStates {
  private abstract static class TurretState implements SubsystemState {
    protected final TurretIO m_turret;

    private TurretState(TurretIO turret) {
      m_turret = turret;
    }
  }

  public static class ClosedLoop extends TurretState {
    public ClosedLoop(TurretIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_turret
          .getMotor()
          .setControl(ControlMode.MotionMagicVoltage, m_turret.getTargetPositionMotorRot());
    }

    public void exit() {}
  }

  public static class Manual extends TurretState {
    public Manual(TurretIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_turret
          .getMotor()
          .setControl(
              ControlMode.VoltageOut,
              m_turret.getManualInput() * RobotInfo.TURRET_INFO.MANUAL_INPUT_TO_VOLTS);
    }

    public void exit() {}
  }

  public static class CharacterizationKS extends TurretState {
    private static boolean hasSeenTargetVelocity = false;
    private static long m_lastTime;

    public CharacterizationKS(TurretIO turret) {
      super(turret);
    }

    public void init() {
      m_turret.setKsTestVolts(0.0);
      m_lastTime = RobotController.getFPGATime();
    }

    public void run() {
      if (Math.abs(m_turret.getMotor().getVelocity().getValueAsDouble())
          > RobotInfo.TURRET_INFO.TEST_KS_VELOCITY_THRESHOLD) {
        hasSeenTargetVelocity = true;
      }
      if (!hasSeenTargetVelocity) {
        double testKsVolts =
            m_turret.getksTestVolts()
                + (RobotController.getFPGATime() - m_lastTime) / 1000.0 / 1000.0 * 0.1;
        m_turret.setKsTestVolts(testKsVolts);
        m_turret.getMotor().setControl(ControlMode.VoltageOut, testKsVolts);
      } else {
        m_turret.getMotor().setControl(ControlMode.VoltageOut, 0.0);
      }
      m_lastTime = RobotController.getFPGATime();
    }

    public void exit() {}
  }

  public static class Off extends TurretState {
    public Off(TurretIO turret) {
      super(turret);
    }

    public void init() {}

    public void run() {
      m_turret.getMotor().setControl(ControlMode.DutyCycleOut, 0);
    }

    public void exit() {}
  }
}
