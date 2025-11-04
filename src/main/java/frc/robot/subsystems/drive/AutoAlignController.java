// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class AutoAlignController {
  private double linearkP = 3.5; // 5.0; // 3.5;
  private static final double linearkD = 0.0; // 3.0;
  private static final double thetakP = 7.0; // 7.0
  private static final double thetakD = 0.0;
  private static final double linearTolerance = 0.01;
  private static final double thetaTolerance = Units.degreesToRadians(1.0);
  private static final double toleranceTime = 0.5;
  private double maxLinearVelocity = Constants.Drivetrain.getMaxLinearVelocity();
  private double maxLinearAcceleration = Constants.Drivetrain.getMaxLinearAcceleration() * 0.4;
  private static final double maxAngularVelocity =
      Constants.Drivetrain.getMaxAngularVelocity() * 0.8;
  private static final double maxAngularAcceleration =
      Constants.Drivetrain.getMaxAngularAcceleration() * 0.8;
  private static final double slowLinearVelocity = 2.25;
  private static final double slowLinearAcceleration = 3.0;
  private static final double slowAngularVelocity = Math.PI / 2.0;
  private static final double slowAngularAcceleration = Math.PI;
  private static final double ffMinRadius = 0.2;
  private static final double ffMaxRadius = 0.8;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Translation2d> feedforwardSupplier;
  private final boolean slowMode;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  private final ProfiledPIDController linearController;
  private final ProfiledPIDController thetaController;
  private final Timer toleranceTimer = new Timer();

  public AutoAlignController(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      boolean slowMode) {
    this.poseSupplier = poseSupplier;
    this.feedforwardSupplier = feedforwardSupplier;
    this.slowMode = slowMode;
    // Set up both controllers
    linearController =
        new ProfiledPIDController(linearkP, 0, linearkD, new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance);
    thetaController =
        new ProfiledPIDController(thetakP, 0, thetakD, new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance);
    toleranceTimer.restart();
    updateConstraints();
    resetControllers(drive);
  }

  public AutoAlignController(
      Drive drive,
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> feedforwardSupplier,
      boolean slowMode,
      double linearP,
      double maxLinearVelocity,
      double maxLinearAcceleration) {
    this.linearkP = linearP;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxLinearVelocity = maxLinearVelocity;
    this.poseSupplier = poseSupplier;
    this.feedforwardSupplier = feedforwardSupplier;
    this.slowMode = slowMode;
    // Set up both controllers
    linearController =
        new ProfiledPIDController(linearkP, 0, linearkD, new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance);
    thetaController =
        new ProfiledPIDController(thetakP, 0, thetakD, new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance);
    toleranceTimer.restart();
    updateConstraints();
    resetControllers(drive);
  }

  private void updateConstraints() {
    if (slowMode) {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(slowLinearVelocity, slowLinearAcceleration));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(slowAngularVelocity, slowAngularAcceleration));
    } else {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));
    }
  }

  private void resetControllers(Drive drive) {
    // Reset measurements and velocities
    Pose2d currentPose = drive.getPose();
    Pose2d goalPose = poseSupplier.get();
    Twist2d fieldVelocity = drive.getfieldVelocity();
    Rotation2d robotToGoalAngle =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity =
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(robotToGoalAngle.unaryMinus())
                .getX());
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  public ChassisSpeeds update(Drive drive) {

    // Control to setpoint
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);
    if (linearController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(linearController.getSetpoint().position, 0.0, new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Reset tolerance timer
    if (!linearController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    // Log data
    Logger.recordOutput("AutoAlign/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoAlign/DistanceSetpoint", linearController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/SetpointPose",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/GoalPose", targetPose);

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation()
            .plus(feedforwardSupplier.get());
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return toleranceTimer.hasElapsed(toleranceTime);
  }
}
