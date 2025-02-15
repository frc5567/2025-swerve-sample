package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Simple command to move the elevator manually for testing Utilizes the DutyCycleOut class to set
 * the power of the elevator motor
 */
public class MoveElevator extends Command {

  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DutyCycleOut m_pctPower;

  /**
   * Primary constructor for the MoveElevator command. Requires the ElevatorSubsystem object be
   * passed in since there should only be one instance of that.
   *
   * @param elevatorSubsystem The one instance of the elevator object
   * @param pctPower The power to run the motor at -- should be Duty Cycle (-1 to 1)
   */
  public MoveElevator(ElevatorSubsystem elevatorSubsystem, DutyCycleOut pctPower) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pctPower = pctPower;

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
   * command will move the elevator motor at the power level specified in the constructor.
   */
  @Override
  public void execute() {
    double currentPosition = m_elevatorSubsystem.getPositionInMillimeters();
    System.out.println(
        "Current Position: [" + currentPosition + "] power [" + m_pctPower.Output + "]");
    m_elevatorSubsystem.moveMechanism(m_pctPower);
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
   * The end routine. Called when the Command ends. This command will stop the elevator motor.
   *
   * @param interrupted True if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopMechanism();
  }
}
