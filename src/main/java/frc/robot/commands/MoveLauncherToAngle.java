package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherAngleSubsystem;

public class MoveLauncherToAngle extends Command {

  private final LauncherAngleSubsystem m_launcherAngleSubsystem;
  private final double m_targetPosition; // distance from start in millimeters

  public MoveLauncherToAngle(LauncherAngleSubsystem launcherAngleSubsystem, double targetPosition) {
    m_launcherAngleSubsystem = launcherAngleSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(launcherAngleSubsystem);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    m_launcherAngleSubsystem.setPositionInPercentTravel(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    double currentPosition = m_launcherAngleSubsystem.getPositionInPercentTravel();
    System.out.println(
        "Current Position: [" + currentPosition + "] Target Position: [" + m_targetPosition + "]");
    // TODO: Need to adjust for proper tolerance.
    return Math.abs(currentPosition - m_targetPosition) < 3;
  }

  @Override
  public void end(boolean interrupted) {
    m_launcherAngleSubsystem.stopLauncherAngle();
  }
}
