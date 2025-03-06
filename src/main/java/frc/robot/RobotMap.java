package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.generated.TunerConstants;

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

  public static final class DriveTrainConstants {

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // 3/4 of a rotation per second max angular velocity
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

  public static final class FieldConstants {
    public static final double FIELD_WIDTH = 8.23;
    public static final double FIELD_LENGTH = 15.85;

    ////

    public static enum FIELD_ELEMENT {
      // 2025 - BLUE APRIL TAGS
      BLUE_CORAL_STATION_LEFT,
      BLUE_CORAL_STATION_RIGHT,
      BLUE_PROCESSOR,
      BLUE_BARGE_FRONT,
      BLUE_BARGE_BACK,
      BLUE_REEF_AB,
      BLUE_REEF_CD,
      BLUE_REEF_EF,
      BLUE_REEF_GH,
      BLUE_REEF_IJ,
      BLUE_REEF_KL,
      // 2025 - RED APRIL TAGS
      RED_CORAL_STATION_LEFT,
      RED_CORAL_STATION_RIGHT,
      RED_PROCESSOR,
      RED_BARGE_FRONT,
      RED_BARGE_BACK,
      RED_REEF_AB,
      RED_REEF_CD,
      RED_REEF_EF,
      RED_REEF_GH,
      RED_REEF_IJ,
      RED_REEF_KL,
      // 2025 - Blue Calculated Positions
      BLUE_CORAL_STATION_LEFT_ALLIANCE,
      BLUE_CORAL_STATION_LEFT_SIDEWALL,
      BLUE_CORAL_STATION_RIGHT_ALLIANCE,
      BLUE_CORAL_STATION_RIGHT_SIDEWALL,
      BLUE_LEFT_CAGE,
      BLUE_RIGHT_CAGE,
      BLUE_CENTER_CAGE,
      BLUE_REEF_CENTER,
      BLUE_REEF_A,
      BLUE_REEF_B,
      BLUE_REEF_C,
      BLUE_REEF_D,
      BLUE_REEF_E,
      BLUE_REEF_F,
      BLUE_REEF_G,
      BLUE_REEF_H,
      BLUE_REEF_I,
      BLUE_REEF_J,
      BLUE_REEF_K,
      BLUE_REEF_L,
      // 2025 - Red Calculated Positions
      RED_CORAL_STATION_LEFT_ALLIANCE,
      RED_CORAL_STATION_LEFT_SIDEWALL,
      RED_CORAL_STATION_RIGHT_ALLIANCE,
      RED_CORAL_STATION_RIGHT_SIDEWALL,
      RED_LEFT_CAGE,
      RED_RIGHT_CAGE,
      RED_CENTER_CAGE,
      RED_REEF_CENTER,
      RED_REEF_A,
      RED_REEF_B,
      RED_REEF_C,
      RED_REEF_D,
      RED_REEF_E,
      RED_REEF_F,
      RED_REEF_G,
      RED_REEF_H,
      RED_REEF_I,
      RED_REEF_J,
      RED_REEF_K,
      RED_REEF_L
    };

    public static enum REEF_OFFSETS {
      LEFT_BRANCH,
      RIGHT_BRANCH
    };

    public static enum TAG_IDS {
      // Blue
      BLUE_CORAL_STATION_LEFT_TAG(13),
      BLUE_CORAL_STATION_RIGHT_TAG(12),
      BLUE_PROCESSOR_TAG(16),
      BLUE_BARGE_FRONT_TAG(14),
      BLUE_BARGE_BACK_TAG(4),
      BLUE_REEF_AB_TAG(18),
      BLUE_REEF_CD_TAG(17),
      BLUE_REEF_EF_TAG(22),
      BLUE_REEF_GH_TAG(21),
      BLUE_REEF_IJ_TAG(20),
      BLUE_REEF_KL_TAG(19),
      // Red
      RED_CORAL_STATION_LEFT_TAG(1),
      RED_CORAL_STATION_RIGHT_TAG(2),
      RED_PROCESSOR_TAG(3),
      RED_BARGE_FRONT_TAG(5),
      RED_BARGE_BACK_TAG(15),
      RED_REEF_AB_TAG(7),
      RED_REEF_CD_TAG(8),
      RED_REEF_EF_TAG(9),
      RED_REEF_GH_TAG(10),
      RED_REEF_IJ_TAG(11),
      RED_REEF_KL_TAG(6);

      private final int value;

      TAG_IDS(int value) {
        this.value = value;
      }

      public int getValue() {
        return value;
      }
      ;
    }

    public static final class FIELD_TAGS {
      // specified here
      // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
      // Do we even need this if we can use AprilTagFieldLayout???
      public static final Pose3d m_aprilTag1 =
          new Pose3d(
              Inches.of(657.37),
              Inches.of(25.8),
              Inches.of(58.5),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(126.0)));
      public static final Pose3d m_aprilTag2 =
          new Pose3d(
              Inches.of(657.37),
              Inches.of(291.2),
              Inches.of(58.5),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(234.0)));
      public static final Pose3d m_aprilTag3 =
          new Pose3d(
              Inches.of(455.15),
              Inches.of(317.15),
              Inches.of(51.25),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(270.0)));
      public static final Pose3d m_aprilTag4 =
          new Pose3d(
              Inches.of(365.20),
              Inches.of(241.64),
              Inches.of(73.54),
              new Rotation3d(Degrees.of(0.0), Degrees.of(30.0), Degrees.of(0.0)));
      public static final Pose3d m_aprilTag5 =
          new Pose3d(
              Inches.of(365.20),
              Inches.of(75.39),
              Inches.of(73.54),
              new Rotation3d(Degrees.of(0.0), Degrees.of(30.0), Degrees.of(0.0)));
      public static final Pose3d m_aprilTag6 =
          new Pose3d(
              Inches.of(530.49),
              Inches.of(130.17),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(300.0)));
      public static final Pose3d m_aprilTag7 =
          new Pose3d(
              Inches.of(546.87),
              Inches.of(158.5),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));
      public static final Pose3d m_aprilTag8 =
          new Pose3d(
              Inches.of(530.49),
              Inches.of(186.83),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(60.0)));
      public static final Pose3d m_aprilTag9 =
          new Pose3d(
              Inches.of(497.77),
              Inches.of(186.83),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(120.0)));
      public static final Pose3d m_aprilTag10 =
          new Pose3d(
              Inches.of(481.39),
              Inches.of(158.5),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(180.0)));
      public static final Pose3d m_aprilTag11 =
          new Pose3d(
              Inches.of(497.77),
              Inches.of(130.17),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(240.0)));
      public static final Pose3d m_aprilTag12 =
          new Pose3d(
              Inches.of(33.51),
              Inches.of(25.8),
              Inches.of(58.5),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(54.0)));
      public static final Pose3d m_aprilTag13 =
          new Pose3d(
              Inches.of(33.51),
              Inches.of(291.2),
              Inches.of(58.5),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(306.0)));
      public static final Pose3d m_aprilTag14 =
          new Pose3d(
              Inches.of(325.68),
              Inches.of(241.64),
              Inches.of(73.54),
              new Rotation3d(Degrees.of(0.0), Degrees.of(30.0), Degrees.of(180.0)));
      public static final Pose3d m_aprilTag15 =
          new Pose3d(
              Inches.of(325.68),
              Inches.of(75.39),
              Inches.of(73.54),
              new Rotation3d(Degrees.of(0.0), Degrees.of(30.0), Degrees.of(180.0)));
      public static final Pose3d m_aprilTag16 =
          new Pose3d(
              Inches.of(235.73),
              Inches.of(-0.15),
              Inches.of(51.25),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(90.0)));
      public static final Pose3d m_aprilTag17 =
          new Pose3d(
              Inches.of(160.39),
              Inches.of(130.17),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(240.0)));
      public static final Pose3d m_aprilTag18 =
          new Pose3d(
              Inches.of(144.0),
              Inches.of(158.5),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(180.0)));
      public static final Pose3d m_aprilTag19 =
          new Pose3d(
              Inches.of(160.39),
              Inches.of(186.83),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(120.0)));
      public static final Pose3d m_aprilTag20 =
          new Pose3d(
              Inches.of(193.1),
              Inches.of(186.83),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(60.0)));
      public static final Pose3d m_aprilTag21 =
          new Pose3d(
              Inches.of(209.49),
              Inches.of(158.5),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));
      public static final Pose3d m_aprilTag22 =
          new Pose3d(
              Inches.of(193.1),
              Inches.of(130.17),
              Inches.of(12.13),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(300.0)));
    }

    public static final class FIELD_TRANSFORMS {
      public static final Transform3d m_leftCoralBranch =
          new Transform3d(
              Inches.of(0.0),
              Inches.of(-6.5),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));
      public static final Transform3d m_rightCoralBranch =
          new Transform3d(
              Inches.of(0.0),
              Inches.of(6.5),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0)));
      public static final Transform3d ROBOT_TRANSFORM =
          new Transform3d(
              Inches.of(13.0),
              Inches.of(0.0),
              Inches.of(0.0),
              new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(180.0)));
    }
  }
}
