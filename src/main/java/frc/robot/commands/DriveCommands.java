// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.AutoAlignController;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private AlignMode currentDriveMode = AlignMode.NORMAL_TELEOP;
  private static AutoAlignController autoAlignController = null;
  private static ChassisSpeeds desiredSpeeds = null;

  // 9c
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return new MainDriveCommand(drive, xSupplier, ySupplier, omegaSupplier);
    // return Commands.run(
    //     () -> {
    //       // Apply deadband
    //       double linearMagnitude =
    //           MathUtil.applyDeadband(
    //               Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    //       Rotation2d linearDirection =
    //           new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    //       double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    //       // Square values
    //       linearMagnitude = linearMagnitude * linearMagnitude;
    //       omega = Math.copySign(omega * omega, omega);

    //       // Calcaulate new linear velocity
    //       Translation2d linearVelocity =
    //           new Pose2d(new Translation2d(), linearDirection)
    //               .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
    //               .getTranslation();

    //       // Convert to field relative speeds & send command
    //       boolean isFlipped = false;
    //       // We're flipping at Blue instead of Red (which was 6328 default)
    //       Optional<Alliance> ally = DriverStation.getAlliance();
    //       if(ally.get() == DriverStation.Alliance.Red){
    //         isFlipped = true;
    //       }
    //         drive.runVelocity(
    //           ChassisSpeeds.fromFieldRelativeSpeeds(
    //               linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
    //               linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
    //               omega * drive.getMaxAngularSpeedRadPerSec(),
    //               isFlipped
    //                   ? drive.getRotation().plus(new Rotation2d(Math.PI))
    //                   : drive.getRotation()));
    //     },
    //     drive);
  }

  public static Command autoAlign(
      Drive drive, boolean leftSide, boolean barging, boolean seekingAlgae) {
    System.out.println("REUESTED--------------------");
    Supplier<Pose2d> onTheFly =
        AutoAlignDesitationDeterminer.destination(drive.getPose(), leftSide, barging, seekingAlgae);
    if (leftSide) {
      drive.m_LimeLight1.setCropY(-1, 1);
      // drive.m_LimeLight1.setMegatag(true);
    } else {
      drive.m_LimeLight1.setCropY(-1, 1);
      // drive.m_LimeLight1.setMegatag(false);
    }
    autoAlignController =
        new AutoAlignController(
            drive,
            onTheFly, // ampAlignedPose,
            () -> {
              return new Translation2d();
            },
            false);

    return Commands.runOnce(
        () -> {
          desiredSpeeds = autoAlignController.update(drive);
          drive.runVelocity(desiredSpeeds);
        },
        drive);
  }

  public void setDriveMode(AlignMode newDriveMode) {
    currentDriveMode = newDriveMode;
  }
}
