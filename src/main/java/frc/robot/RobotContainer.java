// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climber.ClimbCommand;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberWinchSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

  private final Telemetry m_logger = new Telemetry(MaxSpeed);

  private final CommandXboxController m_pilotController = new CommandXboxController(0);

  private final CopilotGamePad m_copilotController = new CopilotGamePad(1);

  private final DutyCycleOut m_upOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut m_downOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut m_intakeOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut m_launchOutput = new DutyCycleOut(0.0);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // public final ElevatorSubsystem m_elevator = new ElevatorSubsystem(29);
  // public final LauncherSubsystem m_launcher = new LauncherSubsystem(30);
  // public final LauncherAngleSubsystem m_launcherAngle = new LauncherAngleSubsystem();
  public final ClimberWinchSubsystem m_climberWinch = new ClimberWinchSubsystem();

  /* Path follower */
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    configureBindings();
  }

  private void registerNamedCommands() {
    // Register Named Commands
    // NamedCommands.registerCommand("MoveElevatorTo180", new MoveElevatorToPosition(elevator,
    // 180));
    // NamedCommands.registerCommand("MoveElevatorTo0", new MoveElevatorToPosition(elevator, 0));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -m_pilotController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_pilotController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_pilotController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    /*
    pilotController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    pilotController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-pilotController.getLeftY(), -pilotController.getLeftX()))));
    */

    /*
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    pilotController
        .back()
        .and(pilotController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    pilotController
        .back()
        .and(pilotController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    pilotController
        .start()
        .and(pilotController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    pilotController
        .start()
        .and(pilotController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Run SysId routines when holding back/start and A/B.
    // Note that each routine should be run exactly once in a single log.
    pilotController
        .back()
        .and(pilotController.b())
        .whileTrue(elevator.sysIdDynamic(Direction.kForward));
    pilotController
        .back()
        .and(pilotController.a())
        .whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    pilotController
        .start()
        .and(pilotController.b())
        .whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    pilotController
        .start()
        .and(pilotController.a())
        .whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    */

    // reset the field-centric heading on left bumper press
    m_pilotController
        .leftBumper()
        .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    // Toggle the arm position
    // pilotController.y().onTrue(new MoveElevatorToPosition(elevator, 600));

    // pilotController.x().onTrue(new MoveElevatorToPosition(elevator, 1000));

    // pilotController.rightBumper().onTrue(new MoveElevatorToPosition(elevator, 0));

    // Test controls for the Launcher
    // pilotController.y().whileTrue(new IntakeCoralCommand(m_launcher, intakeOutput.withOutput(-0.0
    // 2)));
    // pilotController.x().whileTrue(new LaunchCoralCommand(m_launcher,
    // launchOutput.withOutput(0.2)));

    // Test controls for the LauncherAngle mechanism
    // m_pilotController.y().whileTrue(new MoveLauncherToIntakePosition(m_launcherAngle));
    // m_pilotController.x().whileTrue(new MoveLauncherToLaunchPosition(m_launcherAngle));
    // m_pilotController.rightBumper().whileTrue(new MoveLauncherToStartPosition(m_launcherAngle));

    m_pilotController.a().whileTrue(new ClimbCommand(m_climberWinch));

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return m_autoChooser.getSelected();
  }
}
