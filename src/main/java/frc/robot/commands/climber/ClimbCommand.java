package frc.robot.commands.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimberWinchSubsystem;

/**
 * A command that moves the climber to the climb position. The system has a 40:1 gear ratio, 2.25
 * rotations of output shaft for total travel, so 90 complete rotations for complete travel.
 */
public class ClimbCommand extends Command {

  private final ClimberWinchSubsystem m_climber;

  /**
   * Creates a new ClimbCommand command.
   *
   * @param climber The ClimberWinchSubsystem to use
   */
  public ClimbCommand(ClimberWinchSubsystem climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  /**
   * The initialization routine. Called when the Command is first created. This method is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  /**
   * The primary execution routine. Called every time the Command is run by the scheduler. This
   * command will move the climber motor to the finished climb position.
   */
  @Override
  public void execute() {
    m_climber.setPositionInRotations(RobotMap.ClimberConstants.CLIMB_ROTATION_COUNT);
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command will finish when the climber is within
   * tolerance of the target position.
   */
  @Override
  public boolean isFinished() {
    // Finish when the climber is within tolerance of the target position
    Angle rotations = m_climber.getPositionInRotations();
    double currentPosition = rotations.magnitude();
    System.out.println(
        "Current Position: ["
            + currentPosition
            + "] Target Position: ["
            + RobotMap.ClimberConstants.CLIMB_ROTATION_COUNT
            + "]");
    return Math.abs(currentPosition - RobotMap.ClimberConstants.CLIMB_ROTATION_COUNT)
        < RobotMap.ClimberConstants.ROTATION_TOLERANCE;
  }

  /**
   * The end routine. Called when the Command ends. This command will stop the climber motor.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimberWinch();
  }
}
