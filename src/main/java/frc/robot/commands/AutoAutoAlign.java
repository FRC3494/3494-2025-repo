package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.AutoAlignController;
import frc.robot.subsystems.drive.Drive;

public class AutoAutoAlign extends Command {
  Drive drive;
  private Timer timer;
  private double time;

  private  AutoAlignController autoAlignController = null;
  private  ChassisSpeeds desiredSpeeds = null;
  private  Pose2d targetPos = null;

  private double linearKp;
  private double maxLinearAcceleration;
  private double maxLinearVelocity;

  private boolean isCustom = false;

  public AutoAutoAlign(Drive drive, double time, Pose2d targetPos) {
    this.drive = drive;
    this.timer = new Timer();
    
    this.time = time;
    this.targetPos = targetPos;
    addRequirements(drive);
    
  }
  public AutoAutoAlign(Drive drive, double time, Pose2d targetPos, double linearKp, double maxLinearAcceleration, double maxLinearVelocity) {
    this.drive = drive;
    this.timer = new Timer();
    this.time = time;
    this.targetPos = targetPos;
    this.linearKp = linearKp;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxLinearVelocity = maxLinearVelocity;
    this.isCustom = true;
    addRequirements(drive);
    
  }

  @Override
  public void initialize(){
    timer.reset();
    timer.start();
    Supplier<Pose2d> onTheFly = getTargetPos();
    if(isCustom){
        autoAlignController =
            new AutoAlignController(
                drive,
                onTheFly, // ampAlignedPose,
                () -> {
                return new Translation2d();
                },
                false, linearKp, maxLinearVelocity, maxLinearAcceleration);
    }
    else{
        autoAlignController =
            new AutoAlignController(
                drive,
                onTheFly, // ampAlignedPose,
                () -> {
                return new Translation2d();
                },
                false);
        }
  }

  @Override
  public void execute(){
           desiredSpeeds = autoAlignController.update(drive);
          drive.runVelocity(desiredSpeeds);
  }
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(time)) {
            return true;
        }
        return false;

  }
  @Override
  public void end(boolean interupted){
    drive.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    0 * drive.getMaxLinearSpeedMetersPerSec(),
                    0 * drive.getMaxLinearSpeedMetersPerSec(),
                    0* drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));
  }
  public Supplier<Pose2d> getTargetPos(){
    Supplier<Pose2d> targetSupplier = () -> {return targetPos;};
    return targetSupplier;
  }
}
