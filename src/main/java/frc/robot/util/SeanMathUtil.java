package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class SeanMathUtil {
  public static double distance(Pose2d pos1, Pose2d pos2) {
    double xdist = pos2.getX() - pos1.getX();
    double ydist = pos2.getY() - pos1.getY();
    return Math.sqrt(Math.pow(ydist, 2) + Math.pow(xdist, 2));
  }
}
