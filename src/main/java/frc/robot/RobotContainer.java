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
import frc.robot.commands.MoveLauncherToIntakePosition;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LauncherAngleSubsystem;

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
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController pilotController = new CommandXboxController(0);

  private final DutyCycleOut upOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut downOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut intakeOutput = new DutyCycleOut(0.0);
  private final DutyCycleOut launchOutput = new DutyCycleOut(0.0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // public final ElevatorSubsystem m_elevator = new ElevatorSubsystem(29);
  // public final LauncherSubsystem m_launcher = new LauncherSubsystem(30);
  public final LauncherAngleSubsystem m_launcherAngle = new LauncherAngleSubsystem(31);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

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
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -pilotController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -pilotController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -pilotController.getRightX()
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
    pilotController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
    pilotController.y().whileTrue(new MoveLauncherToIntakePosition(m_launcherAngle, 0.0));
    pilotController.x().whileTrue(new MoveLauncherToIntakePosition(m_launcherAngle, 1.8));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
