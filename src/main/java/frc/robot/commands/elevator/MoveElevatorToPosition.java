package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to move the elevator mechanism to a specific position. This command should be used to
 * move the elevator mechanism to a specific position.
 */
public class MoveElevatorToPosition extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;
  private final double m_targetPosition; // distance from start in millimeters

  /**
   * Primary constructor for the MoveElevatorToPosition command. Requires the ElevatorSubsystem
   * object be passed in since there should only be one instance of that.
   *
   * @param elevatorSubsystem The one instance of the elevator object
   * @param targetPosition The position to move the elevator to in millimeters
   */
  public MoveElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_targetPosition = targetPosition;
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
   * command will move the elevator mechanism to the target position specified in the constructor.
   */
  @Override
  public void execute() {
    m_elevatorSubsystem.setPositionInMillimeters(m_targetPosition);
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command will finish when the elevator mechanism is
   * within tolerance of the target position.
   */
  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    double currentPosition = m_elevatorSubsystem.getPositionInMillimeters();
    System.out.println(
        "Current Position: [" + currentPosition + "] Target Position: [" + m_targetPosition + "]");
    return Math.abs(currentPosition - m_targetPosition)
        < RobotMap.ElevatorConstants.POSITION_TOLERANCE;
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
