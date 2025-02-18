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

public class LauncherAngleSubsystem implements Subsystem {

  private TalonFX m_launcherAngleMotor;

  // 5:1 ratio
  // total travel is 2.9166667
  //
  private final double kDistanceInPercentPerRotation = 0.3428;
  private final double kOffset = 0.432617;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // TODO: Must measure and tune these values, preferably under load!!
  private static final Slot0Configs launcherAngleGains =
      new Slot0Configs()
          .withKP(2.5)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  /**
   * Base constructor for the sybsystem. Utilizes a Phoenix 6 Talon FX for motion
   *
   * @param motorPort Can ID of the motor controller to be used within the subsystem
   */
  public LauncherAngleSubsystem(int motorPort) {
    m_launcherAngleMotor = new TalonFX(motorPort);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(5)).withPeakReverseVoltage(Volts.of(-5));
    configs.withSlot0(launcherAngleGains);
    m_launcherAngleMotor.getConfigurator().apply(configs);
  }

  /**
   * getPositionInRotations returns the current position of the system in terms of rotations of the
   * motor/encoder since initialization.
   *
   * @return Angle object representing the rotations of the the motor
   */
  public Angle getPositionInRotations() {
    StatusSignal<Angle> ssAngle = m_launcherAngleMotor.getPosition();
    Angle rotations = ssAngle.getValue();
    Angle offset = Angle.ofRelativeUnits(kOffset, edu.wpi.first.units.Units.Rotations);
    rotations = rotations.plus(offset);
    return rotations;
  }

  /**
   * getPositionInPercentTravel returns the current position of the system in terms of percentage of
   * total possible travel since initialization.
   *
   * @return double representing percentage of travel from 0 (starting position)
   */
  public double getPositionInPercentTravel() {
    Angle curAngle = this.getPositionInRotations();
    double distance = curAngle.magnitude() * kDistanceInPercentPerRotation;
    return distance;
  }

  /**
   * setPositionInPercentTravel drives the subsystem to the target position. We will translate to
   * rotations to set the control using a PositionVoltage object.
   *
   * @param distance The distance from origin to request the system to move in percentage of total
   *     travel. A setpoint.
   */
  public void setPositionInRotations(double rotations) {
    m_launcherAngleMotor.setControl(m_positionVoltage.withPosition(rotations));
  }

  /**
   * stopClimberWinch will stop the climberWinch motor regardless prior calls.
   *
   * @return Does not return a value.
   */
  public void stopLauncherAngle() {
    m_launcherAngleMotor.set(0);
  }

  private void moveLauncherAngle(VoltageOut voltage) {
    m_launcherAngleMotor.setControl(voltage);
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineLauncherAngle =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdLauncherAngle_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> m_launcherAngleMotor.setControl(new VoltageOut(output)), null, this));

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLauncherAngle.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLauncherAngle.dynamic(direction);
  }
}
