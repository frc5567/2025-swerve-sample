package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class GetElevatorPositionCommand extends Command {
  private final ElevatorSubsystem m_elevator;

  public GetElevatorPositionCommand(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Elevator Position: [" + m_elevator.getPositionInMillimeters() + "]");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMechanism();
  }
}
