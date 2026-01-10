// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.frc2025;

import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.elevator.ElevatorIO;
import com.team973.lib.devices.GreyPigeonIO;
import com.team973.lib.util.AllianceCache;
import com.team973.lib.util.Joystick;
import com.team973.lib.util.Logger;
import com.team973.lib.util.SubsystemManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import org.ironmaple.simulation.SimulatedArena;

public class Robot extends TimedRobot {
  private final Logger m_logger = new Logger("robot");

  private final SubsystemManager m_subsystemManager = SubsystemManager.init(m_logger);

  private final GreyPigeonIO m_pigeon = m_subsystemManager.getPigeon();
  private final DriveController m_driveController = m_subsystemManager.getDriveController();
  private final ElevatorIO m_elevator = m_subsystemManager.getElevator();

  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.XboxController, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

  private void syncSensors() {
    m_driveController.syncSensors();
    m_elevator.syncSensors();
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_elevator.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    m_elevator.reset();
  }

  private void log() {
    m_subsystemManager.log();
    m_driveController.log();
    m_elevator.log();
  }

  private void updateJoysticks() {
    m_driverStick.update();
    m_coDriverStick.update();
  }

  public Robot() {
    resetSubsystems();
    m_driveController.startOdometrey();
  }

  @Override
  public void robotPeriodic() {
    syncSensors();
    log();
    updateJoysticks();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_driveController.setRobotIsAutonomous(false);
    m_driveController.setControllerOption(DriveController.ControllerOption.DriveWithJoysticks);
  }

  @Override
  public void teleopPeriodic() {
    double allianceScalar = 1.0;
    if (AllianceCache.Get().get() == Alliance.Red) {
      // Our gyroscope is blue-centric meaning that facing away from the alliance wall
      // is a 0 degree heading. But the driver station is facing 180 when we are on
      // the
      // red alliance. So when we are the red alliance we need to flip the joystick
      // inputs.
      // Ideally we would convert this to polar coordinates, rotate by 180, and then
      // convert
      // back to cartesian. But the algebra here is equivalent to just negating the X
      // and Y
      // so that's what we iwll do for now.
      allianceScalar = -1.0;
    }

    m_driveController
        .getDriveWithJoysticks()
        .updateInput(
            -allianceScalar * m_driverStick.getLeftYAxis() * 0.7,
            allianceScalar * m_driverStick.getLeftXAxis() * 0.7,
            m_driverStick.getRightXAxis() * 0.8);

    if (m_coDriverStick.getAButtonPressed()) {
      m_elevator.setTargetPreset(ElevatorIO.Preset.One);
    } else if (m_coDriverStick.getBButtonPressed()) {
      m_elevator.setTargetPreset(ElevatorIO.Preset.Two);
    } else if (m_coDriverStick.getXButtonPressed()) {
      m_elevator.setTargetPreset(ElevatorIO.Preset.Three);
    } else if (m_coDriverStick.getYButtonPressed()) {
      m_elevator.setState(ElevatorIO.State.Manual);
    }

    m_elevator.setManualInput(m_coDriverStick.getLeftYAxis());

    updateSubsystems();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    m_pigeon.simulationUpdate();
  }
}
