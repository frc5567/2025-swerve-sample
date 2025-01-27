package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RadialActuator implements Subsystem {

  private TalonSRX armMotor;
  // We're dealing with a 10:1 gear ratio, and 4096 ticks per input rev
  private static final double kEncoderTicksPerRevolution = 409600; // adjust based on your encoder
  private static final double kArmMaxAngle = 90.0; // maximum arm angle in degrees

  public RadialActuator(int motorPort) {
    armMotor = new TalonSRX(motorPort);

    // Configure the TalonSRX encoder settings, set the sensor phase if needed
    armMotor.configSelectedFeedbackSensor(
        com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative);
    armMotor.setSensorPhase(true); // Make sure the sensor phase matches your mechanical setup
  }

  public double getArmPosition() {
    // Get position in encoder ticks and convert to degrees
    double ticks = armMotor.getSelectedSensorPosition();
    return ticks * (360.0 / kEncoderTicksPerRevolution); // Convert to degrees
  }

  public double getTicks() {
    // Get position in encoder ticks and convert to degrees
    double ticks = armMotor.getSelectedSensorPosition();
    return ticks;
  }

  public void setArmPosition(double position) {
    // Convert desired position to encoder ticks
    double targetTicks = position * (kEncoderTicksPerRevolution / 360.0);
    armMotor.set(ControlMode.Position, targetTicks);
    double curPos = getArmPosition();
    double ticks = getTicks();
    System.out.println(
        "Attempting to move to position ("
            + position
            + ") Currently ("
            + curPos
            + ") TargetTicks ("
            + targetTicks
            + ") Currently ("
            + ticks
            + ")");
  }

  public void setArmSpeed(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public TalonSRX getArmController() {
    return armMotor;
  }

  public void stopArm() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
