package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
// Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Helpers.LimelightHelpers;
import frc.robot.Helpers.LimelightHelpers.RawFiducial;

public class VisionSubSystem {
  private final NetworkTable limelightTable;

  public VisionSubSystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public Pose2d getAprilTagPose() {
    // Get pose data from LimeLight
    double[] poseData = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

    // Check if pose data is valid
    if (poseData.length < 6) {
      // Return a default pose or handle the error appropriately
      return new Pose2d();
    }

    // Get raw AprilTag/Fiducial data
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double txnc = fiducial.txnc; // X offset (no crosshair)
      double tync = fiducial.tync; // Y offset (no crosshair)
      double ta = fiducial.ta; // Target area
      double distToCamera = fiducial.distToCamera; // Distance to camera
      double distToRobot = fiducial.distToRobot; // Distance to robot
      double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
    }

    // Extract translation and rotation components
    Translation2d translation = new Translation2d(poseData[0], poseData[1]);
    Rotation2d rotation = Rotation2d.fromDegrees(poseData[5]); // Assuming yaw angle is at index 5

    // Create and return the Pose2d object
    return new Pose2d(translation, rotation);
  }
}
