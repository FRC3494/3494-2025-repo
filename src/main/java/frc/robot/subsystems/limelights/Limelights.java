package frc.robot.subsystems.limelights;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class Limelights {
  private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter; // NICE IDEA IMPLEMENT LATER
  private String limelightName;
  private Drive drivetrain;

  private Object[] lastMeasurement = null;

  private boolean validMeasurment = false;
  private double measurementTimeStamp;
  private Pose2d measurementPosition;

  private boolean useMegatag1 = false;
  private LimelightHelpers.PoseEstimate limelightLeftMeasurment;

  public Limelights(Drive drivetrain, String limeLightName) {
    this.drivetrain = drivetrain;
    this.limelightName = limeLightName;
  }

  public void periodic() {
    try {
      // Use MegaTag2 because better?
      // PLEASE FIX BIG PROBLEM, PRETTY SURE IMU ROTATION IS 90 DEGREES OFF, ALSO WHAT DOES THIS
      // FUNCTION DO
      // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2

      LimelightHelpers.SetRobotOrientation(
          limelightName, drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

      limelightLeftMeasurment = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

      if (useMegatag1) {
        limelightLeftMeasurment = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
      }

      boolean leftLimelightEmpty = limelightLeftMeasurment == null;
      boolean rotationRateTooHigh = drivetrain.rotationRate > 4.0 * Math.PI;

      boolean noTagsFound = true;
      boolean tooFarAway = true;

      if (!leftLimelightEmpty) {
        noTagsFound = limelightLeftMeasurment.tagCount() == 0;
        tooFarAway = limelightLeftMeasurment.avgTagDist() > 3.1; // 3.25;
      }

      // logAllFiducialsPose3d(limelightLeftMeasurment);

      try {
        Logger.recordOutput(limelightName + "/noTagsFound", noTagsFound);
        Logger.recordOutput(limelightName + "/tooFarAway", tooFarAway);
        Logger.recordOutput(limelightName + "/limelightLeftMeasurment", limelightLeftMeasurment);
        Logger.recordOutput(limelightName + "/Valid", validMeasurment);
      } catch (Exception e) {
      }

      if (leftLimelightEmpty || rotationRateTooHigh || noTagsFound || tooFarAway) {
        validMeasurment = false;
        return;
      }

      validMeasurment = true;

      measurementTimeStamp = limelightLeftMeasurment.timestampSeconds();
      measurementPosition = limelightLeftMeasurment.pose();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // returns validity of last measurment
  public boolean measurmentValid() {
    return validMeasurment;
  }

  public LimelightHelpers.PoseEstimate getMeasurement() {
    return limelightLeftMeasurment;
  }

  public double getMeasurementTimeStamp() {
    return measurementTimeStamp;
  }

  public Pose2d getMeasuremPosition() {
    return measurementPosition;
  }

  /**
   * Sets the crop window for the camera. The crop window in the UI must be completely open. Notably
   * has the side effect of resetting the X Crop
   *
   * @param minCrop Minimum Y value (-1 to 1)
   * @param maxCrop Maximum Y value (-1 to 1)
   */
  public void setCropY(double minCrop, double maxCrop) {
    LimelightHelpers.setCropWindow(limelightName, -1, 1, minCrop, maxCrop);
  }

  /**
   * Sets the crop window for the camera. The crop window in the UI must be completely open. Notably
   * has the side effect of resetting the X Crop
   *
   * @param megaTagStauts Sets which Megatag We pull from, TRUE = Megatag1, FALSE = Megatag2
   */
  public void setMegatag(boolean megaTagStauts) {
    useMegatag1 = megaTagStauts;
  }
}
