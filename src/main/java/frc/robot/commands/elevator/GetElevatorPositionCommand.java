package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to get the current position of the elevator mechanism. This command should be used to
 * determine the current position of the elevator mechanism.
 */
public class GetElevatorPositionCommand extends Command {
  private final ElevatorSubsystem m_elevator;

  /**
   * Primary constructor for the GetElevatorPositionCommand command. Requires the ElevatorSubsystem
   * object be passed in since there should only be one instance of that.
   *
   * @param elevator The one instance of the elevator object
   */
  public GetElevatorPositionCommand(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
    addRequirements(elevator);
  }

  /**
   * The initiaization routine. Called when the Command is first created. This mehod is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {}

  /**
   * The primary execution routine. Called every time the Command is run by the scheduler. This
   * command will get the current position of the elevator mechanism.
   */
  @Override
  public void execute() {
    System.out.println("Elevator Position: [" + m_elevator.getPositionInMillimeters() + "]");
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
    m_elevator.stopMechanism();
  }
}
