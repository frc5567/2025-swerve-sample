package frc.robot.commands.launcher;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Simple command to engage the launcher motor to expel a coral game piece. This command should
 * eventually be changed to use a VelocityOut signal to control the motor speed.
 */
public class LaunchCoralCommand extends Command {

  private final LauncherSubsystem m_launcher;
  private final DutyCycleOut m_pctPower;

  /**
   * Primary constructor for the LaunchCoralCommand command. Requires the LauncherSubsystem object
   * be passed in since there should only be one instance of that.
   *
   * @param launcher The one instance of the launcher object
   * @param pctPower The power to run the motor at -- should be Duty Cycle (-1 to 1)
   */
  public LaunchCoralCommand(LauncherSubsystem launcher, DutyCycleOut pctPower) {
    m_launcher = launcher;
    m_pctPower = pctPower;

    addRequirements(launcher);
  }

  /**
   * The initiaization routine. Called when the Command is first created. This mehod is kind of
   * redundant since the constructor is doing much the same.
   */
  @Override
  public void initialize() {}

  /**
   * The primary execution routine. Called every time the Command is run by the scheduler. This
   * command will move the launcher motor at the DutyCycle power level specified in the constructor.
   */
  @Override
  public void execute() {
    m_launcher.spinLauncher(m_pctPower);
  }

  /**
   * isFinished is the Completion test. Called every time the Command is run by the scheduler to
   * determine if the command has completed. This command never completes by measurement.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * The end routine. Called when the Command ends. This command will stop the launcher motor.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_launcher.stopLauncher();
  }
}
