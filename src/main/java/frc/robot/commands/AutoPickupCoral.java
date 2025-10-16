package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SeanMathUtil;

public class AutoPickupCoral extends Command {
  private Timer timer;
  Drive drive;
  Arm arm;
  Elevator elevator;
  Intake intake;
  GroundIntake groundIntake;
  private double time;
  public double driveSpeed = -0.25;
  boolean groundIntakeActivated = false;

  public AutoPickupCoral(
      Drive drive,
      GroundIntake groundIntake,
      Arm arm,
      Elevator elevator,
      Intake intake,
      double time) {
    this.time = time;
    this.drive = drive;
    this.groundIntake = groundIntake;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.timer = new Timer();

    addRequirements(drive);
    addRequirements(groundIntake);
    addRequirements(arm);
    addRequirements(elevator);
    addRequirements(intake);
  }

  public AutoPickupCoral(
      Drive drive,
      GroundIntake groundIntake,
      Arm arm,
      Elevator elevator,
      Intake intake,
      double time,
      double driveSpeed) {
    this.time = time;
    this.drive = drive;
    this.groundIntake = groundIntake;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.driveSpeed = driveSpeed;
    this.timer = new Timer();

    addRequirements(drive);
    addRequirements(groundIntake);
    addRequirements(arm);
    addRequirements(elevator);
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
    Logger.recordOutput("Drive/Searching", true);
    elevator.setElevatorPosition(Constants.Presets.liftIntake);
    groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
    intake.setSpeed(-0.75);
    arm.setTargetAngle(
        groundIntake.wanttoPOP
            ? Constants.Presets.armGroundTransferWithPop
            : Constants.Presets.armGroundTransfer,
        0);
  }

  // DOCUMENT SPEED: work slow was: -0.5, and motor torque was 0.3
  @Override
  public void execute() {
    // double driveSpeed = -0.25;
    // if(drivetrain.seesNote() == false){
    //     driveSpeed = 0;
    // }

    boolean atPosition =
        SeanMathUtil.comparePosition(
                arm.getPosition(),
                groundIntake.wanttoPOP
                    ? Constants.Presets.armGroundTransferWithPop
                    : Constants.Presets.armGroundTransfer,
                0.05)
            && SeanMathUtil.comparePosition(elevator.getTicks(), Constants.Presets.liftIntake, 1);
    if (atPosition && !groundIntakeActivated) {
      groundIntake.setIntakePower(-0.85, 0.85);
    }

    System.out.println(time + "|" + timer.hasElapsed(time));
    Logger.recordOutput("Drive/Auto-Timer", timer.get());
    double omegaRot;
    omegaRot = drive.getCoralYaw();
    // drive.runVelocity(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    //     0 * drive.getMaxLinearSpeedMetersPerSec(),
    //     -0.25 * drive.getMaxLinearSpeedMetersPerSec(),
    //     omegaRot * drive.getMaxAngularSpeedRadPerSec(),
    //     false ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    double xspeed = Math.sin(drive.getRotation().getRadians()) * driveSpeed;
    double yspeed = Math.cos(drive.getRotation().getRadians()) * driveSpeed;
    drive.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            xspeed * drive.getMaxLinearSpeedMetersPerSec(),
            yspeed * drive.getMaxLinearSpeedMetersPerSec(),
            omegaRot * drive.getMaxAngularSpeedRadPerSec(),
            drive.getRotation()));
  }

  @Override
  public void end(boolean interupted) {
    System.out.println("ENDING Note COMMAND");
    drive.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            0 * drive.getMaxLinearSpeedMetersPerSec(),
            0 * drive.getMaxLinearSpeedMetersPerSec(),
            0 * drive.getMaxAngularSpeedRadPerSec(),
            drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("Drive/Searching", true);
    if (timer.hasElapsed(time) || groundIntake.getDistanceSensorTripped()) {
      return true;
    }
    // try {
    //     if(drive.getCoralYaw() == 0){
    //         System.out.println("STOPPED BECUASE I DIDNT SEE A CORAL");
    //         return true;
    //     }
    // } catch (JsonProcessingException e) {
    //     // TODO Auto-generated catch block
    //     e.printStackTrace();
    // }
    // if(intake.hasNote()){return true;}
    return false;
  }
}
