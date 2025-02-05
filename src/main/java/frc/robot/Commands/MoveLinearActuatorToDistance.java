package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialLinearActuatorTalonFX;

public class MoveLinearActuatorToDistance extends Command {

  private final RadialLinearActuatorTalonFX m_elevatorSubsystem;
  private final double m_targetPosition; // distance from start in millimeters

  public MoveLinearActuatorToDistance(
      RadialLinearActuatorTalonFX elevatorSubsystem, double targetPosition) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    m_elevatorSubsystem.setPositionInMillimeters(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    double currentPosition = m_elevatorSubsystem.getPositionInMillimeters();
    System.out.println(
        "Current Position: [" + currentPosition + "] Target Position: [" + m_targetPosition + "]");
    return Math.abs(currentPosition - m_targetPosition) < 0.5; // within 5mm tolerance
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopMechanism();
  }
}
