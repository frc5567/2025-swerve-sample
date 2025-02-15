package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class StopElevatorCommand extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;

  public StopElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    m_elevatorSubsystem.stopMechanism();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopMechanism();
  }
}
