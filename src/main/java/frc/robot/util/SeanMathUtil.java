package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class SeanMathUtil {
  public static double distance(Pose2d pos1, Pose2d pos2) {
    double xdist = pos2.getX() - pos1.getX();
    double ydist = pos2.getY() - pos1.getY();
    return Math.sqrt(Math.pow(ydist, 2) + Math.pow(xdist, 2));
  }

  public static boolean comparePosition(double position, double target, double threshold) {
    return Math.abs(position - target) < threshold;
  }

  public static boolean compareArmPosition(double position, double target, boolean greaterThan) {
    if (position > Constants.Arm.forwardSoftLimit) {
      position -= 1;
    }
    if (target > Constants.Arm.forwardSoftLimit) {
      target -= 1;
    }

    if (greaterThan) {
      return position > target;
    } else {
      return position < target;
    }
  }

  public static boolean compareArmPosition(double position, double target, double threshold) {
    if (position > Constants.Arm.forwardSoftLimit) {
      position -= 1;
    }
    if (target > Constants.Arm.forwardSoftLimit) {
      target -= 1;
    }

    return Math.abs(position - target) < threshold;
  }
}
