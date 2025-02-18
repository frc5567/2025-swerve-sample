package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class LauncherSubsystem implements Subsystem {

  private DigitalInput m_coralSensor;
  private TalonFX m_launcherMotor;

  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  // TODO: Must measure and tune these values, preferably under load!!
  private static final Slot0Configs launcherGains =
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
  public LauncherSubsystem(int motorPort) {
    m_coralSensor = new DigitalInput(0);
    m_launcherMotor = new TalonFX(motorPort);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.withSlot0(launcherGains);
    m_launcherMotor.getConfigurator().apply(configs);
  }

  /**
   * haveCoral will return the state of the coral sensor. It should return true if the sensor is
   * triggered indicating we have a coral
   */
  public boolean haveCoral() {
    return !m_coralSensor.get();
  }

  /**
   * stopClimberWinch will stop the climberWinch motor regardless prior calls.
   *
   * @return Does not return a value.
   */
  public void stopLauncher() {
    m_launcherMotor.set(0);
  }

  public void spinLauncher(DutyCycleOut pctPower) {
    m_launcherMotor.setControl(pctPower);
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineLauncher =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdLauncher_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> m_launcherMotor.setControl(new VoltageOut(output)), null, this));

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLauncher.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLauncher.dynamic(direction);
  }
}
