package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherAngleSubsystem;

public class MoveLauncherToIntakePosition extends Command {

  private final LauncherAngleSubsystem m_launchAngle;
  private final double m_targetPosition; // position in percent of total travel. Should be 0-0.58333

  public MoveLauncherToIntakePosition(LauncherAngleSubsystem launchAngle, double targetPosition) {
    m_launchAngle = launchAngle;
    m_targetPosition = targetPosition;
    addRequirements(launchAngle);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    m_launchAngle.setPositionInRotations(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    Angle rotations = m_launchAngle.getPositionInRotations();
    double currentPosition = rotations.magnitude();
    System.out.println(
        "Current Position: [" + currentPosition + "] Target Position: [" + m_targetPosition + "]");
    return Math.abs(currentPosition - m_targetPosition) < 0.02; // within 5mm tolerance
  }

  @Override
  public void end(boolean interrupted) {
    m_launchAngle.stopLauncherAngle();
  }
}
