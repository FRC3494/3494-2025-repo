package frc.robot.subsystems.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import javax.annotation.Nullable;
import org.littletonrobotics.junction.AutoLog;

public class LimelightIO {
  private String limelightName;
  private Drive drivetrain;

  @AutoLog
  public static class LimelightIOInputs {
    public Rotation2d drivetrainHeading;
    public boolean isDrivetrainRotationRateTooHigh;

    public @Nullable LimelightHelpers.PoseEstimate limelightMeasurement;
  }

  public LimelightIO(Drive drivetrain, String limeLightName) {
    this.drivetrain = drivetrain;
    this.limelightName = limeLightName;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.drivetrainHeading = drivetrain.getPose().getRotation();
    inputs.isDrivetrainRotationRateTooHigh = drivetrain.rotationRate > 4.0 * Math.PI;

    // Use MegaTag2 because better?
    // PLEASE FIX BIG PROBLEM, PRETTY SURE IMU ROTATION IS 90 DEGREES OFF, ALSO WHAT DOES THIS
    // FUNCTION DO
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
    LimelightHelpers.SetRobotOrientation(
        this.limelightName, inputs.drivetrainHeading.getDegrees(), 0, 0, 0, 0, 0);

    inputs.limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
  }
}
