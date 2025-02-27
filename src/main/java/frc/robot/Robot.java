// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.sim.PhysicsSim;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private int m_outputCounter = 0;

  private final boolean kUseLimelight = true;

  private Optional<Alliance> m_alliance;

  private PoseEstimate m_curPoseEstimate;

  private final Field2d m_field;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_field = new Field2d();
  }

  @Override
  public void robotInit() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_outputCounter++;
    if (m_outputCounter >= 50) {
      m_outputCounter = 0;
      // Angle curAngle = m_robotContainer.m_launcherAngle.getPositionInRotations();
      // double output = curAngle.magnitude();
      // System.out.println("Launcher Position: Rotations [" + output + "]");
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
      m_alliance = DriverStation.getAlliance();

      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_curPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        } else if (m_alliance.get() == Alliance.Blue) {
          m_curPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        }
      }
      if ((m_curPoseEstimate != null)
          && !((m_curPoseEstimate.pose.getX() == 0) && (m_curPoseEstimate.pose.getY() == 0))) {
        m_robotContainer.m_drivetrain.addVisionMeasurement(
            m_curPoseEstimate.pose, Utils.fpgaToCurrentTime(m_curPoseEstimate.timestampSeconds));
        // System.out.println(
        //     "Robot saw an AprilTag and tried to adjust pose ["
        //         + m_curPoseEstimate.pose.getX()
        //         + "]["
        //         + m_curPoseEstimate.pose.getX()
        //         + "]");
      }
    }
    double curTime = Utils.getCurrentTimeSeconds();
    Optional<Pose2d> curPose = m_robotContainer.m_drivetrain.samplePoseAt(curTime);
    if (curPose.isPresent()) {
      m_field.setRobotPose(curPose.get());
    }
  }

  @Override
  public void disabledInit() {

    // We want the mechanisms to be able to be manually controlled when the robot is disabled
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Coast);
    // m_robotContainer.m_launcherAngle.setBrakeMode(NeutralModeValue.Coast);

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_alliance = DriverStation.getAlliance();

    if (m_alliance.get() == Alliance.Red) {
      System.out.println("We are on the Red Alliance!");
    } else if (m_alliance.get() == Alliance.Blue) {
      System.out.println("We are on the Blue Alliance!");
    } else {
      System.out.println("Alliance is unknown.");
    }

    m_robotContainer.m_drivetrain.seedFieldCentric();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // We want the mechanisms to be able to be manually controlled when the robot is disabled
    // but they need to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
    // m_robotContainer.m_launcherAngle.setBrakeMode(NeutralModeValue.Brake);
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

    m_alliance = DriverStation.getAlliance();

    if (m_alliance.get() == Alliance.Red) {
      System.out.println("We are on the Red Alliance!");
    } else if (m_alliance.get() == Alliance.Blue) {
      System.out.println("We are on the Blue Alliance!");
    } else {
      System.out.println("Alliance is unknown.");
    }

    SignalLogger.start();

    // We want the mechanisms to be able to be manually controlled when the robot is disabled
    // but they need to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
    // m_robotContainer.m_launcherAngle.setBrakeMode(NeutralModeValue.Brake);
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

    m_alliance = DriverStation.getAlliance();
    if (m_alliance.get() == Alliance.Red) {
      System.out.println("We are on the Red Alliance!");
    } else if (m_alliance.get() == Alliance.Blue) {
      System.out.println("We are on the Blue Alliance!");
    } else {
      System.out.println("Alliance is unknown.");
    }

    // We want the mechanisms to be able to be manually controlled when the robot is disabled
    // but they need to be in Brake mode when enabled so it holds position when being driven.
    // m_robotContainer.m_elevator.setBrakeMode(NeutralModeValue.Brake);
    // m_robotContainer.m_launcherAngle.setBrakeMode(NeutralModeValue.Brake);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
