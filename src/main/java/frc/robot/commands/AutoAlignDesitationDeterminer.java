package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class AutoAlignDesitationDeterminer {
    public static Supplier<Pose2d> destination(Pose2d robotPosition, boolean leftSide){
        Supplier<Pose2d> targetSupplier =
        () -> {
            double minDistance = 99999999;
            int minIndex = -1;
            for(int i = 0; i< Constants.Field.Reef.sideLocations.length; i++){
                double distance = getDistance(robotPosition, Constants.Field.Reef.sideLocations[i]);
                if(minDistance == -1){
                    minDistance = distance;
                }
                else if(distance < minDistance){
                    minDistance = distance;
                    minIndex = i;
                }
            }

            Pose2d targetPose = new Pose2d();
            if(leftSide){
               
                 targetPose = Constants.Field.Reef.leftLocations[minIndex];
            }
            else{
          
                 targetPose = Constants.Field.Reef.rightLocations[minIndex];
            }
          return targetPose;
        };
        return targetSupplier;
    }

    public static double getDistance(Pose2d pos1, Translation2d pos2){
        return Math.sqrt((pos1.getX() - pos2.getX())*(pos1.getX() - pos2.getX()) + (pos1.getY() - pos2.getY())*(pos1.getY() - pos2.getY()));
    }
}
