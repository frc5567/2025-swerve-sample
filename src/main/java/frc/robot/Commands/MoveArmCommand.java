package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialActuator;

public class MoveArmCommand extends Command {

    private final RadialActuator armSubsystem;
    private final double speed;  // speed to rotate

    public MoveArmCommand(RadialActuator armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // You could add logic to smooth the movement if necessary
    }

    @Override
    public void execute() {
        armSubsystem.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
