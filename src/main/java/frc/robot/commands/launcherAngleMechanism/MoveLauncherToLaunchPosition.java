package frc.robot.commands.launcherAngleMechanism;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.LauncherAngleSubsystem;

/**
 * A command that moves the launcher to the intake position. The system has a 45:1 gear ratio, 8
 * degrees of output rotation per 360 degrees of motor rotation. Launch position is 120 degrees, or
 * 15 motor rotations
 */
public class MoveLauncherToLaunchPosition extends Command {

  private final LauncherAngleSubsystem m_launchAngle;

  /**
   * Creates a new MoveLauncherToLaunchPosition command.
   *
   * @param launchAngle The launcher angle subsystem to use
   */
  public MoveLauncherToLaunchPosition(LauncherAngleSubsystem launchAngle) {
    m_launchAngle = launchAngle;
    addRequirements(launchAngle);
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
   * command will move the launcher motor to the intake position.
   */
  @Override
  public void execute() {
    m_launchAngle.setPositionInRotations(RobotMap.AngleMotorConstants.LAUNCH_ROTATION_COUNT);
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command will finish when the launcher is within a
   * tolerance of the target position.
   */
  @Override
  public boolean isFinished() {
    // Finish when the arm is within a tolerance of the target position
    Angle rotations = m_launchAngle.getPositionInRotations();
    double currentPosition = rotations.magnitude();
    System.out.println(
        "Current Position: ["
            + currentPosition
            + "] Target Position: ["
            + RobotMap.AngleMotorConstants.LAUNCH_ROTATION_COUNT
            + "]");
    return Math.abs(currentPosition - RobotMap.AngleMotorConstants.LAUNCH_ROTATION_COUNT)
        < RobotMap.AngleMotorConstants.ROTATION_TOLERANCE;
  }

  /**
   * The end routine. Called when the Command ends. This command will stop the launcher motor.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_launchAngle.stopLauncherAngle();
  }
}
