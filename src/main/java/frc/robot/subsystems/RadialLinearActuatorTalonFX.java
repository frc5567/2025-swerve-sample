package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * Sample class to approximate what needs to be done for a
 * radial actuator or elevator that is controlling
 * something that ends up moving linearly
 */
public class RadialLinearActuatorTalonFX implements Subsystem {

  private TalonFX m_elevatorMotor;
  // 2 inch gear will cause 159.59 mm travel per rotation
  private final double kDistanceInMillimetersPerRotation = 360;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private static final Slot0Configs actuatorGains =
      new Slot0Configs()
          .withKP(2.5)
          .withKI(0)
          .withKD(0)
          .withKS(0.001)
          .withKV(0.11)
          .withKA(0.003)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  /**
   * Base constructor for the sybsystem. Utilizes a Phoenix 6 Talon FX for motion
   *
   * @param motorPort Can ID of the motor controller to be used within the subsystem
   */
  public RadialLinearActuatorTalonFX(int motorPort) {
    m_elevatorMotor = new TalonFX(motorPort);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.withSlot0(actuatorGains);
    m_elevatorMotor.getConfigurator().apply(configs);
  }

  /**
   * getPositionInRotations returns the current position of the system in terms of rotations of the
   * motor/encoder since initialization.
   *
   * @return Angle object representing the rotations of the the motor
   */
  private Angle getPositionInRotations() {
    StatusSignal<Angle> ssAngle = m_elevatorMotor.getPosition();
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
    m_elevatorMotor.setControl(m_positionVoltage.withPosition(desiredRotations));
  }

  /** stopMechanism will simply command the motor controller to not move any more. */
  public void stopMechanism() {
    m_elevatorMotor.set(0);
  }

  private void moveMechanism(VoltageOut voltage) {
    m_elevatorMotor.setControl(voltage);
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineActuator =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdActuator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> m_elevatorMotor.setControl(new VoltageOut(output)), null, this));

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineActuator.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineActuator.dynamic(direction);
  }
}
