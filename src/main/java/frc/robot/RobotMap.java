package frc.robot;

/**
 * The RobotMap is a mapping of all of the constants used in the robot program. This includes motor
 * ports, sensor ports, and other constants. This should be the only place in the code where numbers
 * are used directly.
 */
public class RobotMap {

  public static final class AngleMotorConstants {
    public static final int LAUNCHER_ANGLE_MOTOR_PORT = 31;

    public static final double OFFSET = -0.051270;

    public static final double ROTATION_TOLERANCE = 0.25;

    public static final double INITIAL_ROTATION_COUNT = 0.1;

    public static final double INTAKE_ROTATION_COUNT = 9.25;

    public static final double LAUNCH_ROTATION_COUNT = 15.5;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_WINCH_MOTOR_PORT = 32;

    public static final double OFFSET = -5.281250;

    public static final double INITIAL_ROTATION_COUNT = 0.1;

    public static final double CLIMB_ROTATION_COUNT = 300.0;

    public static final double ROTATION_TOLERANCE = 1.0;
  }

  public static final class ElevatorConstants {

    public static final double POSITION_TOLERANCE = 10.0;
  }
}
