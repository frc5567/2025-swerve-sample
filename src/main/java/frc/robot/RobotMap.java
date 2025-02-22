package frc.robot;

/**
 * The RobotMap is a mapping of all of the constants used in the robot program. This includes motor
 * ports, sensor ports, and other constants. This should be the only place in the code where numbers
 * are used directly.
 */
public class RobotMap {

  public static final class AngleMotorConstants {
    public static final int LAUNCHER_ANGLE_MOTOR_PORT = 0;

    // TODO: find correct value of tolerance in rotations.
    public static final double ROTATION_TOLERANCE = 0.25;

    public static final double INITIAL_ROTATION_COUNT = 15.0;

    public static final double INTAKE_ROTATION_COUNT = 7.5;

    public static final double LAUNCH_ROTATION_COUNT = 15.0;
  }
}
