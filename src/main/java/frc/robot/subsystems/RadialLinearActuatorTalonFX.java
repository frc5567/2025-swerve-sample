package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

/*
 * Sample class to approximate what needs to be done for a
 * radial actuator or elevator that is controlling
 * something that ends up moving linearly
 */
public class RadialLinearActuatorTalonFX implements Subsystem {

  private TalonFX elevatorMotor;
  // 2 inch gear will cause 159.59 mm travel per rotation
  private final double kDistanceInMillimetersPerRotation = 360;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  /**
   * Base constructor for the sybsystem. Utilizes a Phoenix 6 Talon FX for motion
   *
   * @param motorPort Can ID of the motor controller to be used within the subsystem
   */
  public RadialLinearActuatorTalonFX(int motorPort) {
    elevatorMotor = new TalonFX(motorPort);
  }

  /**
   * getPositionInRotations returns the current position of the system in terms of rotations of the
   * motor/encoder since initialization.
   *
   * @return Angle object representing the rotations of the the motor
   */
  private Angle getPositionInRotations() {
    StatusSignal<Angle> ssAngle = elevatorMotor.getPosition();
    Angle rotations = ssAngle.getValue();
    return rotations;
  }

  /**
   * getPositionInMillimeters returns the current position of the system in terms of millimeters of
   * linear travel since initialization.
   *
   * @return double representing millimeters of travel from 0 (starting position)
   */
  public double getPositionInMillimeters() {
    Angle curAngle = this.getPositionInRotations();
    double distance = curAngle.magnitude() * kDistanceInMillimetersPerRotation;
    return distance;
  }

  /**
   * setPositionInMillimeters drives the subsystem to the target position. We will translate to
   * rotations to set the control using a PositionVoltage object.
   *
   * @param distance The distance from origin to request the system to move. A setpoint.
   */
  public void setPositionInMillimeters(double distance) {
    double desiredRotations = distance / kDistanceInMillimetersPerRotation;
    elevatorMotor.setControl(m_positionVoltage.withPosition(desiredRotations));
  }

  /** stopMechanism will simply command the motor controller to not move any more. */
  public void stopMechanism() {
    elevatorMotor.set(0);
  }
}
