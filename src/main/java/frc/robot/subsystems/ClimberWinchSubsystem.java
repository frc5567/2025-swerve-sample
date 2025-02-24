package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

/**
 * ClimberWinchSubsystem is the subsystem that controls the climber winch motor for the robot. This
 * subsystem is responsible for moving the motor to the correct position to climb the cage
 */
public class ClimberWinchSubsystem implements Subsystem {

  private TalonFX m_climberWinchMotor;

  // 40:1 ratio -- 2.25 rotations of output shaft for total travel
  // so 90 complete rotations for complete travel
  // Start position is 0 rotations

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // TODO: Must measure and tune these values, preferably under load!!
  private static final Slot0Configs climberWinchGains =
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
   * <p>Takes no parameters, as the motor port is defined in the RobotMap
   */
  public ClimberWinchSubsystem() {
    m_climberWinchMotor = new TalonFX(RobotMap.ClimberConstants.CLIMBER_WINCH_MOTOR_PORT);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(2)).withPeakReverseVoltage(Volts.of(-2));
    configs.withSlot0(climberWinchGains);
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_climberWinchMotor.getConfigurator().apply(configs);
  }

  /**
   * getPositionInRotations returns the current position of the system in terms of rotations of the
   * motor/encoder since initialization.
   *
   * @return Angle object representing the rotations of the the motor
   */
  public Angle getPositionInRotations() {
    StatusSignal<Angle> ssAngle = m_climberWinchMotor.getPosition();
    Angle rotations = ssAngle.getValue();
    System.out.print("Climber Position: Rotations [" + rotations.magnitude() + "]");
    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.ClimberConstants.OFFSET, edu.wpi.first.units.Units.Rotations);
    rotations = rotations.plus(offset);
    System.out.println("Offset Rotations [" + rotations.magnitude() + "]");
    return rotations;
  }

  /**
   * setPositionInRotations will move the climberWinch motor to the desired position in rotations of
   * the winch motor (not the output).
   *
   * @param rotations The desired position in rotations.
   */
  public void setPositionInRotations(double rotations) {
    double desiredRotations = rotations - RobotMap.ClimberConstants.OFFSET;
    m_climberWinchMotor.setControl(m_positionVoltage.withPosition(desiredRotations));
  }

  /**
   * stopClimberWinch will stop the climberWinch motor regardless prior calls.
   *
   * @return Does not return a value.
   */
  public void stopClimberWinch() {
    m_climberWinchMotor.set(0);
  }

  /**
   * moveClimberWinch will move the climber winch motor to the desired voltage.
   *
   * @param voltage The power to move the motor -- expressed in a value from -12 to 12.
   */
  private void moveClimberWinch(VoltageOut voltage) {
    m_climberWinchMotor.setControl(voltage);
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineClimberWinch =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdClimberWinch_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> m_climberWinchMotor.setControl(new VoltageOut(output)), null, this));

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineClimberWinch.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineClimberWinch.dynamic(direction);
  }
}
