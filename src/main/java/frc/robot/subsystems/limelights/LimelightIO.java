package frc.robot.subsystems.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import javax.annotation.Nullable;
import org.littletonrobotics.junction.AutoLog;

public class LimelightIO {
  private Drive drivetrain;

  @AutoLog
  public static class LimelightIOInputs {
    public Rotation2d drivetrainHeading;
    public boolean isDrivetrainRotationRateTooHigh;

    public @Nullable LimelightHelpers.PoseEstimate leftLimelightMeasurement;
    public @Nullable LimelightHelpers.PoseEstimate rightLimelightMeasurement;
  }

  public LimelightIO(Drive drivetrain) {
    this.drivetrain = drivetrain;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.drivetrainHeading = drivetrain.getPose().getRotation();
    inputs.isDrivetrainRotationRateTooHigh = drivetrain.rotationRate > 4.0 * Math.PI;

    LimelightHelpers.SetRobotOrientation(
        Constants.Limelight.LEFT_LIMELIGHT_NAME,
        inputs.drivetrainHeading.getDegrees(),
        0,
        0,
        0,
        0,
        0);

    LimelightHelpers.SetRobotOrientation(
        Constants.Limelight.RIGHT_LIMELIGHT_NAME,
        inputs.drivetrainHeading.getDegrees(),
        0,
        0,
        0,
        0,
        0);

    inputs.leftLimelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.Limelight.LEFT_LIMELIGHT_NAME);
    inputs.rightLimelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.Limelight.RIGHT_LIMELIGHT_NAME);
  }
}
