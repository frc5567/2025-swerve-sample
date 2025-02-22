package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherAngleSubsystem;

/**
 * Command to move the launcher to a specific position. This command should be used to move the
 * launcher a different position expressed in rotations of the motor.
 */
public class MoveLauncherToPosition extends Command {

  private final LauncherAngleSubsystem m_launchAngle;
  private final double m_targetPosition; // position in percent of total travel. Should be 0-0.58333

  /**
   * Primary constructor for the MoveLauncherToPosition command. Requires the LauncherAngleSubsystem
   * object be passed in since there should only be one instance of that.
   *
   * @param launchAngle The one instance of the launcher angle object
   * @param targetPosition The position to move the launcher to in rotations
   */
  public MoveLauncherToPosition(LauncherAngleSubsystem launchAngle, double targetPosition) {
    m_launchAngle = launchAngle;
    m_targetPosition = targetPosition;
    addRequirements(launchAngle);
  }

  /**
   * The initiaization routine. Called when the Command is first created. This mehod is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {}

  /**
   * The primary execution routine. Called every time the Command is run by the scheduler. This
   * command will move the launcherAngle mechanism to the target position specified in the
   * constructor.
   */
  @Override
  public void execute() {
    m_launchAngle.setPositionInRotations(m_targetPosition);
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command will finish when the launcherAngle
   * mechanism is within tolerance of the target position.
   */
  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    Angle rotations = m_launchAngle.getPositionInRotations();
    double currentPosition = rotations.magnitude();
    System.out.println(
        "Current Position: [" + currentPosition + "] Target Position: [" + m_targetPosition + "]");
    return Math.abs(currentPosition - m_targetPosition) < 0.02; // within 5mm tolerance
  }

  /**
   * The end routine. Called when the Command ends. This command will stop the launcherAngle
   * mechanism.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_launchAngle.stopLauncherAngle();
  }
}
