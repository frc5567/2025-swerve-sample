package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

/**
 * LauncherAngleSubsystem is the subsystem that controls the launcher angle motor for the robot.
 * This subsystem is responsible for moving the motor to the correct angle to launch a coral game
 * piece.
 */
public class LauncherAngleSubsystem implements Subsystem {

  private TalonFX m_launcherAngleMotor;

  // 45:1 Gear ratio
  // 8 degrees of output rotation per 360 degrees of motor rotation
  // Starting position is 0 degrees
  // Intake position is 60 degrees, or 7.5 motor rotations
  // Launch position is 120 degrees, or 15 motor rotations

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
   * <p>Takes no parameters, as the motor port is defined in the RobotMap
   */
  public LauncherAngleSubsystem() {
    m_launcherAngleMotor = new TalonFX(RobotMap.AngleMotorConstants.LAUNCHER_ANGLE_MOTOR_PORT);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(2)).withPeakReverseVoltage(Volts.of(-2));
    configs.withSlot0(launcherAngleGains);
    m_launcherAngleMotor.getConfigurator().apply(configs);
  }

  /**
   * setBrakeMode sets the brake mode of the motor controller to either coast or brake. Need to
   * expose this so we can set it to coast in disabledInit so the launcherAngle can be manually
   * controlled when the bot is disabled.
   *
   * @param mode the NeutralModeValue to set the motor controller to
   */
  public void setBrakeMode(NeutralModeValue mode) {
    m_launcherAngleMotor.setNeutralMode(mode);
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
    System.out.print("Launcher Position: Rotations [" + rotations.magnitude() + "]");
    Angle offset =
        Angle.ofRelativeUnits(
            RobotMap.AngleMotorConstants.OFFSET, edu.wpi.first.units.Units.Rotations);
    rotations = rotations.plus(offset);
    System.out.println("Offset Rotations [" + rotations.magnitude() + "]");
    return rotations;
  }

  /**
   * setPositionInRotations drives the subsystem to the target position. We will translate to
   * rotations to set the control using a PositionVoltage object.
   *
   * @param rotations The rotations to request the system to move. A setpoint.
   */
  public void setPositionInRotations(double rotations) {
    rotations -= RobotMap.AngleMotorConstants.OFFSET;
    m_launcherAngleMotor.setControl(m_positionVoltage.withPosition(rotations));
  }

  /**
   * stopLauncherAngle will stop the climberWinch motor regardless prior calls.
   *
   * @return Does not return a value.
   */
  public void stopLauncherAngle() {
    m_launcherAngleMotor.set(0);
  }

  /**
   * moveLauncherAngle will move the launcher angle motor to the desired voltage.
   *
   * @param output The power to move the motor -- expressed in a value from -1 to 1.
   */
  private void moveLauncherAngle(DutyCycleOut output) {
    m_launcherAngleMotor.setControl(output);
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
