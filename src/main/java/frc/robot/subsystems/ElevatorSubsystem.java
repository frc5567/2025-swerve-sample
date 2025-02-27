package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * ElevatorSubsystem is the subsystem that controls the elevator motor for the robot. This subsystem
 * is responsible for moving the elevator to the correct position to score a coral game piece.
 */
public class ElevatorSubsystem implements Subsystem {

  private TalonFX m_elevatorMotor;

  /* Position offset to account for the starting position of the elevator */
  private double m_positionOffset = -0.011230;

  /**
   * kDistanceInMillimetersPerRotation is the distance the elevator travels per rotation of the
   * motor. This is calculated by the circumference of the sprocket multiplied by the gear ratio.
   *
   * <p>Sprocket has 2.638 inch diameter (so multiplied by pi) 8.2875 inches circumference. That
   * converts to 210.5mm per rotation of lower stage. Total travel of scoring mechanism is double
   * that at 411mm per rotation (since both stages move together and proportionally the same
   * distance) Gear Ratio is 40:1 411 / 40 = 10.275mm of travel per rotation of motor
   */
  private final double kDistanceInMillimetersPerRotation = 10.477;

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  /**
   * Elevator gains for default slot 0. Note that these values were not tuned and are just
   * placeholders
   */
  private static final Slot0Configs elevatorGains =
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
   * @param motorPort CAN ID of the motor controller to be used within the subsystem
   */
  public ElevatorSubsystem(int motorPort) {
    m_elevatorMotor = new TalonFX(motorPort);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    configs.withSlot0(elevatorGains);
    m_elevatorMotor.getConfigurator().apply(configs);
  }

  /**
   * setBrakeMode sets the brake mode of the motor controller to either coast or brake. Need to
   * expose this so we can set it to coast in disabledInit so the elevator can be manually
   * controlled when the bot is disabled.
   *
   * @param mode the NeutralModeValue to set the motor controller to
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_elevatorMotor.setNeutralMode(mode);
  }

  /**
   * getPositionInRotations returns the current position of the system in terms of rotations of the
   * motor/encoder since initialization.
   *
   * @return Angle object representing the rotations of the the motor
   */
  public Angle getPositionInRotations() {
    StatusSignal<Angle> ssAngle = m_elevatorMotor.getPosition();
    Angle rotations = ssAngle.getValue();

    Angle offset = Angle.ofRelativeUnits(m_positionOffset, edu.wpi.first.units.Units.Rotations);
    rotations = rotations.plus(offset);
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
   * rotations to set the control using a MotionMagicVoltage object.
   *
   * @param distance The distance from origin to request the system to move. A setpoint.
   */
  public void setPositionInMillimeters(double distance) {
    double desiredRotations = distance / kDistanceInMillimetersPerRotation;
    m_elevatorMotor.setControl(m_motionMagicVoltage.withPosition(desiredRotations));
    System.out.println(
        "Setting position to ["
            + distance
            + "] mm Currently ["
            + this.getPositionInMillimeters()
            + "] mm");
  }

  /** stopMechanism stops the elevator motor from moving */
  public void stopMechanism() {
    m_elevatorMotor.set(0);
  }

  /**
   * moveMechanism moves the elevator motor at the specified power (DutyCycle -- ie fractional value
   * from -1 to 1)
   *
   * @param pctPower Value between -1 and 1 that represents the percent power to the motor
   */
  public void moveMechanism(DutyCycleOut pctPower) {
    System.out.println("Setting power to [" + pctPower.Output + "]");
    m_elevatorMotor.setControl(pctPower);
  }

  /**
   * SysId routine for characterizing the elevator. This is used to find PID gains for the elevator
   * motor. This is the Template routine which will be called either for Dynamic or Quasistatic
   * testing. Output is written to the CTRE Signal Logger Details here:
   * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/index.html
   */
  private final SysIdRoutine m_sysIdRoutineElevator =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
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
    return m_sysIdRoutineElevator.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineElevator.dynamic(direction);
  }
}
