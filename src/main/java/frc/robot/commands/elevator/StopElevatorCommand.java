package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to stop the elevator mechanism. This command should be used to stop the elevator
 * mechanism when it is in motion.
 */
public class StopElevatorCommand extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;

  /**
   * Primary constructor for the StopElevatorCommand command. Requires the ElevatorSubsystem object
   * be passed in since there should only be one instance of that.
   *
   * @param elevatorSubsystem The one instance of the elevator object
   */
  public StopElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  /**
   * The initiaization routine. Called when the Command is first created. This mehod is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {}

  /**
   * The primary execution routine. Called every time the Command is run by the scheduler. This
   * command will stop the elevator mechanism.
   */
  @Override
  public void execute() {
    m_elevatorSubsystem.stopMechanism();
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command will always finish immediately.
   */
  @Override
  public boolean isFinished() {
    return true;
  }

  /**
   * The end routine. Called when the Command ends. This command will stop the elevator mechanism.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopMechanism();
  }
}
