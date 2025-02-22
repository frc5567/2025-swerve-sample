// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private int m_outputCounter = 0;

  // TODO: Set this to true in order to use the limelight
  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_outputCounter++;
    if (m_outputCounter >= 50) {
      m_outputCounter = 0;
      Angle curAngle = m_robotContainer.m_launcherAngle.getPositionInRotations();
      double output = curAngle.magnitude();
      System.out.println("Launcher Position: Rotations [" + output + "]");
    }

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null) {
        m_robotContainer.m_drivetrain.addVisionMeasurement(
            llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.m_drivetrain.seedFieldCentric();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // We want the elevator to be able to be manually controlled when the robot is disabled
    // but it needs to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.m_drivetrain.seedFieldCentric();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SignalLogger.start();

    // We want the elevator to be able to be manually controlled when the robot is disabled
    // but it needs to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    SignalLogger.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    // We want the elevator to be able to be manually controlled when the robot is disabled
    // but it needs to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    // PhysicsSim.getInstance().addTalonSRX(m_robotContainer.arm.getArmController(), 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
