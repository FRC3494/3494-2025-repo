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

  public static DriveMode DRIVE_MODE = DriveMode.NORMAL;

  public static enum DriveMode {
    NORMAL,
    DEMO, // Demo without autoalign
    DEMO_AUTOALIGN, // Demo with autoalign
    TRAINING, // Training new drivers
    TRAINING_AUTOALIGN, // Training new drivers with autoalign
  }

  public static boolean getAutoAlignEnabled() {
    return switch (Constants.DRIVE_MODE) {
      case DEMO, TRAINING -> false;
      default -> true;
    };
  }

  public static class Mitocandria {
    public static final int MITOCANDRIA_CAN_ID = 50;

    public static final int LIMELIGHT_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_5VB;
    public static final int PIGEON_CHANNEL = MitoCANdria.MITOCANDRIA_CHANNEL_ADJ;
  }

  public static class Presets {
    public static double globalArmOffset = -0.005; // -0.01
    public static double armIntake = 0.823 - 0.756; // 0.822; // -34.0; //TODO
    public static double armIntakeLow = 0.829 - 0.756; // TODO
    public static double armIntakeAlt = 0.845 - 0.756; // 0.830 //TODO
    public static double armIntakeLowLow = 0.846 - 0.756; // TODO

    public static double armSafePosition = 0.72 - 0.756 + 1;
    public static double armProcessor = 0.54 - 0.756 + 1; // -136.0; //TODO
    public static double armOuttakeL1 = 0.875 - 0.756; // TODO
    public static double armOuttakeL2 = 0.599 - 0.756 + 1; // 0.843
    public static double armOuttakeL3 = 0.619 - 0.756 + 1;
    public static double armGroundTransfer = 0.945 - 0.756 + 0.011; // 0.198
    public static double armGroundTransferWithPop = 0.948 - 0.756 + 0.011; // 0.201

    public static double armAlgeaL2 =
        0.603 - 0.756; // 0.6125; // 0.605// TODO 0.62; difference is -0.027 + 0.05
    public static double armAlgeaL3 = 0.620 - 0.756 + 1;
    public static double armBargeYeet = 0.633 - 0.756 + 1; // TODO
    public static double armBargeStore = 0.833 - 0.756; // TODO

    public static double liftIntake = 0;
    public static double liftIntakeAlt = 10; // 2.238
    public static double liftOuttakeL2 = 20;
    public static double liftOuttakeL3 = 44.5;
    public static double liftAlgeaL3 = 30;
    public static double liftAlgeaL3Auto = 37.0; // 35.643;//
    public static double liftAlgeaL2 = 3.7;
    public static double liftAlgeaTeleopL2 = 0;

    public static double armLoliPop = 0.847 - 0.756;

    public static double liftClimb = 30;
    public static double armClimb = 0.56 - 0.756 + 1;

    public static double climberStage0 = 0.0;
    public static double climberStage1 = -28.0;
    public static double climberStage2 = -53.405;
    public static double climberHover = -36.0;

    public static double armBargeYeetRelease = 0.788 - 0.756; // 0.8075f// 0.8;//0.7965;// TODO

    public static double L1armtest = 0.613 - 0.756 + 1; // 0.610
    public static double L1elevatorTest = 8.38;

    public static double groundIntakeIntake = 0.07; // 0.0;//-19.6;
    public static double groundIntakeHover = 0.07;
    // 2.0;//-17;
    public static double groundIntakePop = 0.07; // 2.0;//-17;
    public static double defenseDelay = 0.0;
    public static double groundIntakeStore = 0.31; // 28.0;//6;
    public static double groundIntakeL1 = 0.36; // not bad: 0.36
    public static double groundIntakeL1High = 0.29;
    public static double groundIntakeStation = 0.31; // 0.3238

    public static double groundIntakeJerk = 0.27; // 0.275; // not bad: 0.26   // 23.0;;// 3.28;
  }

  public static class OI {
    public static int PRIMARY_CONTROLLER_PORT = 0;
  }

  public static class Elevator {
    public static int bottomMagSensorDIO = 9;
    public static int leaderMotor = 12; // 12;
    public static int followerMotor = 13; // 13;
  }

  public static class GroundIntake {
    public static int pivotMotor = 7;
    public static int frontIntakeMotor = 11;
    public static int backIntakeMotor = 10;
    public static int distanceSensorDeviceNumber = 31;

    public static double CoralDistanceTheshold = 400;

    public static double upPIDRange = 0.7;
    public static double downPIDRange = 0.7;

    public static double safePosition = 0.114;
  }

  public static class Arm {
    public static int armMotor = 15;
    public static double manualPowerPOS = 0.006;
    public static double normalPIDRange = 0.9;

    public static double absoluteEncoderOffset = 0.7609175 + 0.006;

    public static int normalCurrentLimit = 75;

    public static double safePosition = 0.777 - 0.756;

    public static double reverseSoftLimit = 0.960 - 0.756;
    public static double forwardSoftLimit = 0.530 - 0.756 + 1;
  }

  public static class Intake {
    public static int intakeMotor = 14;
    public static double DEADBAND = 0.05;

    public static double getSpeedScalar() {
      return switch (DRIVE_MODE) {
        case NORMAL -> 1.0;
        case DEMO, DEMO_AUTOALIGN -> 0.4;
        case TRAINING, TRAINING_AUTOALIGN -> 1.0;
      };
    }
  }

  public static class Climber {
    public static int CLIMBER_MOTOR_CAN_ID = 6;
  }

  public static class Drivetrain {
    public static double L1autoAlignOffset = -0.1;

    public static double L1_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
    public static double L2_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // TODO

    public static double rotationPower(double rawRotationPower) {
      return Math.copySign(Math.pow(rawRotationPower, 2), rawRotationPower);
    }

    public static double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    public static final double trackWidthX = 0.5222; // TODO: random Number
    public static final double trackWidthY = 0.574675;

    public static double getMaxLinearVelocity() {
      return switch (DRIVE_MODE) {
        case DEMO_AUTOALIGN -> 1.0;
        default -> 4.5; // TODO: I made this number up
      };
    }

    public static double getMaxLinearAcceleration() {
      return switch (DRIVE_MODE) {
        case DEMO_AUTOALIGN -> 1.0;
        default -> 9.0; // TODO: I made this number up
      };
    }

    public static double getMaxAngularVelocity() {
      return switch (DRIVE_MODE) {
        case DEMO_AUTOALIGN -> 3.0;
        default -> 6.0; // TODO: I made this number up
      };
    }

    public static double getMaxAngularAcceleration() {
      return switch (DRIVE_MODE) {
        case DEMO_AUTOALIGN -> 5.0;
        default -> 12.0; // TODO: I made this number up
      };
    }

    public static final int PIGEON_PORT = 52;
    public static final int FRONT_LEFT_DRIVE_ID = 18; // 18
    public static final int FRONT_LEFT_STEER_ID = 16; // 16
    public static final int FRONT_LEFT_TURN_ENCODER_ID =
        3; // 3public static final double FRONT_LEFT_OFFSET = Math.toRadians(22);
    public static final double FRONT_LEFT_OFFSET = 1.259;

    public static final int FRONT_RIGHT_DRIVE_ID = 19; // 19
    public static final int FRONT_RIGHT_STEER_ID = 17; // 17
    public static final int FRONT_RIGHT_TURN_ENCODER_ID = 2; // 2
    public static final double FRONT_RIGHT_OFFSET = 2.602;

    public static final int BACK_LEFT_DRIVE_ID = 30; // 30
    public static final int BACK_LEFT_STEER_ID = 2; // 2
    public static final int BACK_LEFT_TURN_ENCODER_ID = 1; // 1
    public static final double BACK_LEFT_OFFSET = 0.248;

    public static final int BACK_RIGHT_DRIVE_ID = 1; // 1
    public static final int BACK_RIGHT_STEER_ID = 3; // 3
    public static final int BACK_RIGHT_TURN_ENCODER_ID = 0; // 0
    public static final double BACK_RIGHT_OFFSET = 2.768;

    public static final double getSpeedScalar() {
      return switch (Constants.DRIVE_MODE) {
        case NORMAL -> 1.0;
        case DEMO, DEMO_AUTOALIGN -> 0.5;
        case TRAINING, TRAINING_AUTOALIGN -> 0.75;
      };
    }

    public static final double getRotationSpeedStationaryScalar() {
      return switch (Constants.DRIVE_MODE) {
        case NORMAL, DEMO_AUTOALIGN -> 0.3;
        case DEMO -> 0.3;
        case TRAINING, TRAINING_AUTOALIGN -> 0.3;
      };
    }

    public static final double getRotationSpeedNormalScalar() {
      return switch (Constants.DRIVE_MODE) {
        case NORMAL, DEMO_AUTOALIGN -> 0.6;
        case DEMO -> 0.3;
        case TRAINING, TRAINING_AUTOALIGN -> 0.6;
      };
    }

    public static final double getRotationSpeedFastScalar() {
      return switch (Constants.DRIVE_MODE) {
        case NORMAL, DEMO_AUTOALIGN -> 1.0;
        case DEMO -> 0.6;
        case TRAINING, TRAINING_AUTOALIGN -> 1.0;
      };
    }
  }

  public static class LEDs {
    public static final int LED_PWM_ID = 0;

    public enum LEDLightPattern {
      // PWM values: https://docs.revrobotics.com/rev-crossover-products/blinkin/gs/patterns
      DISABLED(0.1),
      NONE(0.65), // Solid Orange
      HAS_GAMEPIECE(0.75), // Solid Dark Green
      DEPOSITED(0.69), // Solid Yellow
      INTAKING(0.59); // Solid Dark Red

      public final double value;

      private LEDLightPattern(double value) {
        this.value = value;
      }
    }
  }

  public static class Field {
    public static final double fieldLength =
        Units.inchesToMeters(690.875); // 57 ft 6 and 7/8 in // 17.548 m
    public static final double fieldWidth = Units.inchesToMeters(317); // 26ft 6in //8.0518 m
    public static final Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), fieldWidth);
    public static final Pose2d bargeSpot = new Pose2d(6.68, 5.20, Rotation2d.fromDegrees(-48.83));

    // new Pose2d(7.4, 6.893, new Rotation2d(Math.toRadians(-135.0)));

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
        // new Translation2d(1.172, 1.00), // 8
        // new Translation2d(1.172, 7.0) // 9
      };

      public static final Pose2d[]
          leftLocations = { // Placeholder currently using the center positions
        new Pose2d(3.178, 4.202, Rotation2d.fromDegrees(-90)), // 1
        new Pose2d(
            4.05,
            5.15,
            new Rotation2d(
                -Math.PI / 3.0 - Math.PI / 2.0)), // 2 //4.040, 5.193 //beofre iri4.05, 5.15
        new Pose2d(
            3.78,
            3.080,
            new Rotation2d(Math.PI / 3.0 - Math.PI / 2.0)), // 3 //before iri3.78, 3.080
        new Pose2d(5.028, 5.257, Rotation2d.fromDegrees(150)), // 4
        new Pose2d(5.23, 3.03, new Rotation2d(2 * Math.PI / 3.0 - Math.PI / 2.0)), // 5
        new Pose2d(5.72, 4.17, new Rotation2d(Math.PI - Math.PI / 2.0)), // 6
        new Pose2d(6.001, 0.489, new Rotation2d(Math.PI / 2.0 - Math.PI / 2.0)), // 7
        // new Pose2d(1.312, 0.948, new Rotation2d(0.3 * Math.PI - Math.PI / 2.0)), // 8
        // new Pose2d(1.276, 7.124, new Rotation2d(-0.3 * Math.PI - Math.PI / 2.0)) // 9
      };

      public static final Pose2d[]
          rightLocations = { // Placeholder currently using the center positions
        new Pose2d(3.172, 3.884, Rotation2d.fromDegrees(-90)), // 1 //BEFORE IRI was 3.256, 3.894
        new Pose2d(3.76, 5.035, new Rotation2d(-Math.PI / 3.0 - Math.PI / 2.0)), // 2 //3.76, 5.035
        new Pose2d(
            3.988,
            2.888,
            new Rotation2d(
                Math.PI / 3.0
                    - Math.PI
                        / 2.0)), // 3 //before iri WHATTT  |old 3.988, 2.888 \when revert 3.96, 2.86
        new Pose2d(5.250, 5.065, Rotation2d.fromDegrees(150)), // 4
        new Pose2d(
            4.98,
            2.88,
            new Rotation2d(2 * Math.PI / 3.0 - Math.PI / 2.0)), // 5 before iri 4.98, 2.88
        new Pose2d(5.71, 3.85, new Rotation2d(Math.PI - Math.PI / 2.0)), // 6
        new Pose2d(6.001, 0.489, new Rotation2d(Math.PI / 2.0 - Math.PI / 2.0)), // 7
        // new Pose2d(1.702, 0.613, new Rotation2d(0.3 * Math.PI - Math.PI / 2.0)), // 8
        // new Pose2d(0.685, 6.703, new Rotation2d(-0.3 * Math.PI - Math.PI / 2.0)) // 9
      };
      public static final Translation2d reefCenter = new Translation2d(4.59, 4.026);
    }
  }

  public static class Auto {
    public static class AmpMidAuto {
      public static Pose2d posSwing = new Pose2d(3.0, 1.75, new Rotation2d(Math.toRadians(-90)));
      public static Pose2d pos1 = new Pose2d(2.2, 3.184, new Rotation2d(Math.toRadians(-90)));
      public static Pose2d algeaPluck =
          new Pose2d(3.33, 3.894, new Rotation2d(Math.toRadians(-90)));
      public static Pose2d pos2 = new Pose2d(3.3, 3.894, new Rotation2d(0.0 - Math.PI / 2.0));
      public static Pose2d pos3 = new Pose2d(1.780, 2.703, new Rotation2d(Math.toRadians(-50)));
      public static Pose2d pos4 = new Pose2d(2.041, 4.0, new Rotation2d(Math.toRadians(-90)));
      public static Pose2d pos5 = new Pose2d(3.256, 4.222, new Rotation2d(Math.toRadians(-90)));
    }

    public static class AmpMidAutoRed {
      public static Pose2d posSwing =
          new Pose2d(14.54, 6.302, new Rotation2d(Math.toRadians(-90 + 180)));
      public static Pose2d pos1 =
          new Pose2d(15.34505, 4.8678, new Rotation2d(Math.toRadians(-90 + 180)));
      public static Pose2d algeaPluck =
          new Pose2d(14.21505, 4.1578, new Rotation2d(Math.toRadians(-90 + 180)));
      public static Pose2d pos2 =
          new Pose2d(14.24505, 4.1578, new Rotation2d(0.0 - Math.PI / 2.0 + Math.PI));
      public static Pose2d pos3 =
          new Pose2d(15.76505, 5.3488, new Rotation2d(Math.toRadians(-50 + 180)));
      public static Pose2d pos4 =
          new Pose2d(15.50405, 4.0518, new Rotation2d(Math.toRadians(-90 + 180)));
      public static Pose2d pos5 =
          new Pose2d(14.28905, 3.8298, new Rotation2d(Math.toRadians(-90 + 180)));
    }

    public static class ThreePeiceCoolAuto {
      public static Pose2d pos1 =
          new Pose2d(5.23, 5.02, new Rotation2d(-2 * Math.PI / 3.0 - Math.PI / 2.0)); // 4
      public static Pose2d pos2 = new Pose2d(3.689, 6.393, new Rotation2d(Math.toRadians(-104.8)));
      public static Pose2d pos3 =
          new Pose2d(
              4.05, 5.15, new Rotation2d(-Math.PI / 3.0 - Math.PI / 2.0)); // 2 //4.040, 5.193
      public static Pose2d pos4 = new Pose2d(3.008, 5.986, new Rotation2d(Math.toRadians(-117.30)));
      public static Pose2d pos5 =
          new Pose2d(3.760, 5.035, new Rotation2d(-Math.PI / 3.0 - Math.PI / 2.0)); // 2
      // pos6 is handled by path planner
      public static Pose2d pos7 =
          new Pose2d(6.636, 7.040, new Rotation2d(Math.toRadians(-29.7 - 90)));
      // public static Pose2d pos1 = new Pose2d(4.98, 2.88, new Rotation2d(2 * Math.PI / 3.0 -
      // Math.PI / 2.0));
      // public static Pose2d pos2 = new Pose2d(3.430, 1.547, new
      // Rotation2d(Math.toRadians(-82.03)));
      // public static Pose2d pos3 = new Pose2d(3.78, 3.080, new Rotation2d(Math.PI / 3.0 - Math.PI
      // / 2.0)), // 3
    }
  }
}
