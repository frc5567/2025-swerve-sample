package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevator extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DutyCycleOut m_pctPower;

  public MoveElevator(ElevatorSubsystem elevatorSubsystem, DutyCycleOut pctPower) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pctPower = pctPower;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double currentPosition = m_elevatorSubsystem.getPositionInMillimeters();
    System.out.println(
        "Current Position: [" + currentPosition + "] power [" + m_pctPower.Output + "]");
    m_elevatorSubsystem.moveMechanism(m_pctPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopMechanism();
  }
}
