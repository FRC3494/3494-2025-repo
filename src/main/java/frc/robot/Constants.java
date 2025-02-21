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

package frc.robot;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final int POWER_DISTRIBUTION_PANEL_CAN_ID = 28;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Mitocandria {
    public static final int MITOCANDRIA_CAN_ID = 50;

    public static final int LIMELIGHT_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_5VB;
    public static final int PIGEON_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_ADJ;
  }

  public static class Presets {
    public static double globalArmOffset = -0.04; // -0.01
    public static double armIntake = 0.66; // -34.0;
    public static double armCoral = 0.7; // -136.0;

    public static double armOuttakeL1 = 0.77;
    public static double armOuttakeL2 = 0.224; // -111.0;
    public static double armOuttakeL3 = 0.26; // -106.0;

    public static double armAlgeaL2 = 0.25;
    public static double armAlgeaL3 = 0.23; // -116.0;

    public static double liftIntake = 0;
    public static double liftOuttakeL2 = 20;
    public static double liftOuttakeL3 = 44.0;
  }

  public static class OI {
    public static int PRIMARY_CONTROLLER_PORT = 0;
  }

  public static class Elevator {
    public static int bottomMagSensorDIO = 9;
    public static int leaderMotor = 12;
    public static int followerMotor = 13;
  }

  public static class Arm {
    public static int armMotor = 15;
    public static double manualPowerPOS = 0.006;
  }

  public static class Intake {
    public static int intakeMotor = 14;
    public static double DEADBAND = 0.05;
  }

  public static class Climber {
    public static int CLIMBER_MOTOR_CAN_ID = 6;
  }

  public static class Limelight {
    public static String RIGHT_LIMELIGHT_NAME = "limelight-right";
    public static String LEFT_LIMELIGHT_NAME = "limelight-left";
  }

  public static class Drivetrain {
    public static double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    public static final double trackWidthX = 0.5222; // TODO: random Number
    public static final double trackWidthY = 0.574675;

    public static final double maxLinearVelocity = 4.5; // TODO: I made this number up
    public static final double maxLinearAcceleration = 20.0; // TODO: I made this number up
    public static final double maxAngularVelocity = 6;
    public static final double maxAngularAcceleration = 12;

    public static final int PIGEON_PORT = 52;
    public static final int FRONT_LEFT_DRIVE_ID = 18; // 18
    public static final int FRONT_LEFT_STEER_ID = 16; // 16
    public static final int FRONT_LEFT_TURN_ENCODER_ID = 3; // 3
    public static final double FRONT_LEFT_OFFSET = Math.toRadians(72);

    public static final int FRONT_RIGHT_DRIVE_ID = 19; // 19
    public static final int FRONT_RIGHT_STEER_ID = 17; // 17
    public static final int FRONT_RIGHT_TURN_ENCODER_ID = 2; // 2
    public static final double FRONT_RIGHT_OFFSET = Math.toRadians(151);

    public static final int BACK_LEFT_DRIVE_ID = 30; // 30
    public static final int BACK_LEFT_STEER_ID = 2; // 2
    public static final int BACK_LEFT_TURN_ENCODER_ID = 1; // 1
    public static final double BACK_LEFT_OFFSET = Math.toRadians(13.1);

    public static final int BACK_RIGHT_DRIVE_ID = 1; // 1
    public static final int BACK_RIGHT_STEER_ID = 3; // 3
    public static final int BACK_RIGHT_TURN_ENCODER_ID = 0; // 0
    public static final double BACK_RIGHT_OFFSET = Math.toRadians(223.9);
  }

  public static class Field {
    public static final double fieldLength = Units.inchesToMeters(690.875); // 57 ft 6 and 7/8 in
    public static final double fieldWidth = Units.inchesToMeters(317); // 26ft 6in
    public static final Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

    public static class Reef {
      //       6
      //     ----
      //  4 /    \  5
      //   /      \
      //   \      /
      // 2  \    / 3
      //     ----
      //      1
      // Algea is 7
      // Intake Station Left is 8
      // Intake Station Right is 9
      // NOTE: These side location aren't used to drive to they are just use to search for the
      // nearest side (also these are for the blue side)
      public static final Translation2d[] sideLocations = {
        new Translation2d(3.156, 4.030), // 1
        new Translation2d(3.823, 5.154), // 2
        new Translation2d(3.776, 2.858), // 3
        new Translation2d(5.164, 5.156), // 4
        new Translation2d(5.170, 2.858), // 5
        new Translation2d(5.832, 4.047), // 6
        new Translation2d(6.001, 0.489), // 7
        new Translation2d(1.172, 1.00), // 8
        new Translation2d(1.172, 7.0) // 9
      };

      public static final Pose2d[] leftLocations = { //Placeholder currently using the center positions
        new Pose2d(3.277, 4.176, new Rotation2d(0.0-Math.PI/2.0)), //1
        new Pose2d(4.040, 5.124, new Rotation2d(-Math.PI/3.0-Math.PI/2.0)),  //2
        new Pose2d(3.742, 3.103,new Rotation2d(Math.PI/3.0-Math.PI/2.0)), //3
        new Pose2d(4.943, 5.116,new Rotation2d(-2*Math.PI/3.0-Math.PI/2.0)), //4
        new Pose2d(5.233, 3.095,new Rotation2d(2*Math.PI/3.0-Math.PI/2.0)), //5
        new Pose2d(5.678, 4.164, new Rotation2d(Math.PI-Math.PI/2.0)),//6
        new Pose2d(6.001, 0.489, new Rotation2d(Math.PI/2.0-Math.PI/2.0)),//7
        new Pose2d(1.312, 0.948, new Rotation2d(0.3*Math.PI-Math.PI/2.0)),//8
        new Pose2d(1.101, 7.042, new Rotation2d(-0.3*Math.PI-Math.PI/2.0))//9
      };

      public static final Pose2d[] rightLocations = { //Placeholder currently using the center positions
        new Pose2d(3.326, 3.862, new Rotation2d(0.0-Math.PI/2.0)), //1
        new Pose2d(3.781, 4.964, new Rotation2d(-Math.PI/3.0-Math.PI/2.0)),  //2
        new Pose2d(4.029, 2.930,new Rotation2d(Math.PI/3.0-Math.PI/2.0)), //3
        new Pose2d(5.223, 4.952,new Rotation2d(-2*Math.PI/3.0-Math.PI/2.0)), //4
        new Pose2d(4.946, 2.910,new Rotation2d(2*Math.PI/3.0-Math.PI/2.0)), //5
        new Pose2d(5.663, 3.851, new Rotation2d(Math.PI-Math.PI/2.0)),//6
        new Pose2d(6.001, 0.489, new Rotation2d(Math.PI/2.0-Math.PI/2.0)),//7
        new Pose2d(1.702, 0.613, new Rotation2d(0.3*Math.PI-Math.PI/2.0)),//8
        new Pose2d(1.101, 7.375, new Rotation2d(-0.3*Math.PI-Math.PI/2.0))//9

      };
    }
  }
}
