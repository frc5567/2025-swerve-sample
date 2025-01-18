package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RadialActuator;

public class MoveArmToPositionCommand extends Command {

    private final RadialActuator armSubsystem;
    private final double targetPosition;  // position in degrees

    public MoveArmToPositionCommand(RadialActuator armSubsystem, double targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // You could add logic to smooth the movement if necessary
    }

    @Override
    public void execute() {
        armSubsystem.setArmPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Finish when the arm is within a tolerance of the target position
        double currentPosition = armSubsystem.getArmPosition();
        return Math.abs(currentPosition - targetPosition) < 3.0;  // within 1 degree tolerance
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
