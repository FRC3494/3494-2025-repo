// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

// import frc.robot.subsystems.drive.DriveMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.enums.Direction;
import frc.robot.subsystems.drive.Drive;

public class WheelRadiusCharacterization extends Command {
  private static final double characterizationSpeed = 2.0;
  private static final double driveRadius = Constants.Drivetrain.driveBaseRadius();
  private static DoubleSupplier gyroYawRadsSupplier =
      () -> 0; // needs to be initialized but gets changed later

  private final Drive drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;
  Timer timer = new Timer();

  public WheelRadiusCharacterization(Drive drive, Direction omegaDirection) {
    this.drive = drive;

    timer.start();
    this.omegaDirection = omegaDirection;
    gyroYawRadsSupplier = () -> drive.getPose().getRotation().getRadians();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    System.out.println("started wheel charecterization command");
    timer.reset();
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    System.out.println("Running Wheel Charecterization");
    // Run drive at velocity
    if (omegaDirection == Direction.CLOCKWISE) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0, 0, -1 * characterizationSpeed, drive.getRotation()));
    } else if (omegaDirection == Direction.COUNTER_CLOCKWISE) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0, 0, 1 * characterizationSpeed, drive.getRotation()));
    }

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    // drive.endCharacterization();
    // DriveCommands.currentDriveMode = DriveMode.WHEEL_RADIUS_CHARECTERIZATION;
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }

  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(120)) {
      System.out.println("STOPPPPPPPPPPPPPPPPPPPP");
      return true;
    }
    System.out.println(timer.get() + "||" + accumGyroYawRads);
    return false;
  }
}
// 7
