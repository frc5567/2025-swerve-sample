package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialActuator;

public class MoveArmToPositionCommand extends Command {

  private final RadialActuator m_armSubsystem;
  private final double m_targetPosition; // position in degrees

  public MoveArmToPositionCommand(RadialActuator armSubsystem, double targetPosition) {
    m_armSubsystem = armSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    m_armSubsystem.setArmPosition(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    double currentPosition = m_armSubsystem.getArmPosition();
    return Math.abs(currentPosition - m_targetPosition) < 3.0; // within 1 degree tolerance
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopArm();
  }
}
