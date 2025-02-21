package frc.robot.subsystems.limelights;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Limelights {
  private LimelightIOInputsAutoLogged inputs;
  private LimelightIO limelightIO;

  public Limelights(Drive drivetrain) {
    inputs = new LimelightIOInputsAutoLogged();
    limelightIO = new LimelightIO(drivetrain);
  }

  @SuppressWarnings("null")
  public void periodic(SwerveDrivePoseEstimator poseEstimator) {
    limelightIO.updateInputs(inputs);
    Logger.processInputs("Limelights", inputs);

    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

    if (inputs.isDrivetrainRotationRateTooHigh) return;

    if (inputs.leftLimelightMeasurement != null) {
      poseEstimator.addVisionMeasurement(
          inputs.leftLimelightMeasurement.pose(),
          inputs.leftLimelightMeasurement.timestampSeconds());
    }

    if (inputs.rightLimelightMeasurement != null) {
      poseEstimator.addVisionMeasurement(
          inputs.rightLimelightMeasurement.pose(),
          inputs.rightLimelightMeasurement.timestampSeconds());
    }
  }
}
