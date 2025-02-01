package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialActuator;

public class StopArmCommand extends Command {

  private final RadialActuator armSubsystem;
  private final double speed; // speed to rotate

  public StopArmCommand(RadialActuator armSubsystem) {
    this.armSubsystem = armSubsystem;
    this.speed = 0;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    // You could add logic to smooth the movement if necessary
  }

  @Override
  public void execute() {
    armSubsystem.stopArm();
    double curPos = armSubsystem.getArmPosition();
    double ticks = armSubsystem.getTicks();
    System.out.println("Currently (" + curPos + ") Currently (" + ticks + ")");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }
}
