package frc.robot.commands;

import java.security.Timestamp;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
//This file is unused. I'll deal with it later
public class MainDriveCommand extends Command{
    Drive drive;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier omegaSupplier;
    private double DEADBAND = 0.1;
    Translation2d linearVelocity;
    double timeAtLastRead;
    Timer timer = new Timer();
    Translation2d pastLinearVelocity;
    Twist2d currentLinearTwist;
    double currentLinearVelocity;
    
    public MainDriveCommand(Drive drive,DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier){
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        timer.start();
        addRequirements(drive);
        

    }
    @Override
    public void execute(){
        // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          currentLinearTwist = drive.getfieldVelocity();
          currentLinearVelocity =Math.sqrt((currentLinearTwist.dx * currentLinearTwist.dx) + (currentLinearTwist.dx * currentLinearTwist.dx));
          // Convert to field relative speeds & send command
          boolean isFlipped = false;
          // We're flipping at Blue instead of Red (which was 6328 default)
          Optional<Alliance> ally = DriverStation.getAlliance();
          if(ally.get() == DriverStation.Alliance.Red){
            isFlipped = true;
          }
            drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        if(linearVelocity != null && pastLinearVelocity != null){

          double targetVelocity = Math.sqrt((linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()) + (linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()*linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()));
          Logger.recordOutput("Drive/CurrentLinearV", (currentLinearVelocity));
          Logger.recordOutput("Drive/TargetLinearV", targetVelocity);
          Logger.recordOutput("Drive/LinearAcceleration", (linearVelocity.getNorm()-currentLinearVelocity)/(timer.get()-timeAtLastRead));
        }
        pastLinearVelocity = linearVelocity;  
        timeAtLastRead = timer.get();    
    }
    
}
