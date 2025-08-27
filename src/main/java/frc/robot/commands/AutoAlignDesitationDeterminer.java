package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AutoAlignDesitationDeterminer {
  public static boolean placingAtL1 = false;

  public static Supplier<Pose2d> destination(
      Pose2d robotPosition, boolean leftSide, boolean barging, boolean seekingAlgae) {
    Supplier<Pose2d> targetSupplier =
        () -> {
          double distance;
          double minDistance = 99999999;
          Optional<Alliance> ally = DriverStation.getAlliance();
          int minIndex = -1;
          for (int i = 0; i < Constants.Field.Reef.sideLocations.length; i++) {

            if (ally.get() == DriverStation.Alliance.Red) {
              distance =
                  getDistance(robotPosition, transform2red(Constants.Field.Reef.sideLocations[i]));
            } else {
              distance = getDistance(robotPosition, Constants.Field.Reef.sideLocations[i]);
            }
            if (minDistance == -1) {
              minDistance = distance;
            } else if (distance < minDistance) {
              minDistance = distance;
              minIndex = i;
            }
          }

          Pose2d targetPose = new Pose2d();
          if (leftSide) {
            targetPose = Constants.Field.Reef.leftLocations[minIndex];

          } else {

            targetPose = Constants.Field.Reef.rightLocations[minIndex];
          }
          if (placingAtL1) { // Rotates us to auto align to the other side if we are placing at L1,
            // need to test if we can use the same reef postions or if I need to
            // offset us a bit due to how accurate the odo is :)
            targetPose =
                new Pose2d(
                    targetPose.getX(),
                    targetPose.getY(),
                    new Rotation2d(targetPose.getRotation().getRadians() + Math.PI));
            Translation2d targetTrans = new Translation2d(targetPose.getX(), targetPose.getY());
            Translation2d distanceFromReefCenter =
                Constants.Field.Reef.reefCenter.minus(targetTrans);
            targetTrans =
                targetTrans.plus(
                    distanceFromReefCenter.times(Constants.Drivetrain.L1autoAlignOffset));
            targetPose =
                new Pose2d(targetTrans, new Rotation2d(targetPose.getRotation().getRadians()));
          } else if (seekingAlgae) { // if we want algea just average the left and righ positions
            Pose2d leftPos = Constants.Field.Reef.leftLocations[minIndex];
            Pose2d rightPos = Constants.Field.Reef.rightLocations[minIndex];
            Pose2d averagePos =
                new Pose2d(
                    (leftPos.getX() + rightPos.getX()) / 2.0,
                    (leftPos.getY() + rightPos.getY()) / 2.0,
                    new Rotation2d(
                        (leftPos.getRotation().getRadians() + rightPos.getRotation().getRadians())
                            / 2.0));
            targetPose = averagePos;
          }

          if (barging) {
            targetPose = Constants.Field.bargeSpot;
          }
          if (ally.get() == DriverStation.Alliance.Red) {
            targetPose = pose2red(targetPose);
          }
          return targetPose;
        };
    return targetSupplier;
  }

  public static double getDistance(Pose2d pos1, Translation2d pos2) {
    return Math.sqrt(
        (pos1.getX() - pos2.getX()) * (pos1.getX() - pos2.getX())
            + (pos1.getY() - pos2.getY()) * (pos1.getY() - pos2.getY()));
  }

  public static Translation2d transform2red(Translation2d transform1) {
    Translation2d fieldCenter =
        new Translation2d(Constants.Field.fieldLength, Constants.Field.fieldWidth);
    double redXpos = fieldCenter.getX() - transform1.getX();
    double redYpos = fieldCenter.getY() - transform1.getY();
    return new Translation2d(redXpos, redYpos);
  }

  public static Pose2d pose2red(Pose2d pose1) {
    Translation2d fieldCenter =
        new Translation2d(Constants.Field.fieldLength, Constants.Field.fieldWidth);
    double redXpos = fieldCenter.getX() - pose1.getX();
    double redYpos = fieldCenter.getY() - pose1.getY();
    double redTheta = pose1.getRotation().getRadians() + Math.PI;
    return new Pose2d(redXpos, redYpos, new Rotation2d(redTheta));
  }
}
