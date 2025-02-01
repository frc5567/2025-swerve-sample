package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialLinearActuatorTalonFX;

public class StopLinearActuatorCommand extends Command {

  private final RadialLinearActuatorTalonFX m_elevatorSubsystem;

  public StopLinearActuatorCommand(RadialLinearActuatorTalonFX elevatorSubsyste) {
    m_elevatorSubsystem = elevatorSubsyste;
    addRequirements(elevatorSubsyste);
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
