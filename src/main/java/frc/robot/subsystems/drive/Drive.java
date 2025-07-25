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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.AutoAlignDesitationDeterminer;
import frc.robot.subsystems.limelights.Limelights;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.SeanMathUtil;

public class Drive extends SubsystemBase {

  // public boolean canReadTags = false;
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  public Limelights m_LimeLight1 = new Limelights(this, "limelight-right");
  public Limelights m_LimeLight2 = new Limelights(this, "limelight-left");
  public Limelights m_LimeLight3 = new Limelights(this, "limelight-swerve");
  public Limelights m_LimeLight4 = new Limelights(this, "limelight-barge");

  public double rotationRate = 0;
  public boolean specialPoseEstimation = false;
  public boolean coralIntededforL1 = false;
  public double reefRadiusToSpecialPoseActivation = 3.0;
  double currentRadiusFromReef;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    SparkMaxOdometryThread.getInstance().start();
    // THIS IS JUNK DONT LE THIS RUN, IT OTTA BE OVER RUN BY THE FOLLOWING LOOP
    RobotConfig config =
        new RobotConfig(
            100,
            10,
            new ModuleConfig(
                2,
                20,
                DRIVE_BASE_RADIUS,
                new DCMotor(
                    TRACK_WIDTH_Y,
                    TRACK_WIDTH_X,
                    MAX_LINEAR_SPEED,
                    MAX_ANGULAR_SPEED,
                    DRIVE_BASE_RADIUS,
                    0),
                1,
                1,
                0),
            0);

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as neededb
      e.printStackTrace();
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController( // PPHolonomicController is the built in path
            // following controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config,
        () -> false,
        // DriverStation.getAlliance().isPresent()
        //     && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    m_LimeLight1.periodic();
    m_LimeLight2.periodic();
    m_LimeLight3.periodic();
    m_LimeLight4.periodic();
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    SparkMaxOdometryThread odo = SparkMaxOdometryThread.getInstance();

    // As the queue can change size between this call and the following standard err logging calls
    // this value does not represent the actual amount of errors logged, just how many at this
    // point in the program
    Logger.recordOutput("SparkMaxOdometryThread/DriveErrorCount", odo.pastDriveErrors.size());

    // odo.pastDriveErrors
    //     .iterator()
    //     .forEachRemaining(
    //         (err) -> {
    //           System.err.println("Drive Spark Max error: " + err.toString());
    //         });
    // odo.pastTurnErrors
    //     .iterator()
    //     .forEachRemaining(
    //         (err) -> {
    //           System.err.println("Turn Spark Max error: " + err.toString());
    //         });

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Make sure we own odometry at this point
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    Logger.recordOutput("Odometry Sample Output Length", sampleTimestamps.length);

    odometryLock.unlock();

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      // rawGyroRotation = gyroInputs.yawPosition; // Seans test FIX NOT PERMENATE
      rotationRate = gyroInputs.yawVelocityRadPerSec; // Logged for megaTag

      // Apply update
      // System.out.println(rawGyroRotation+ "|" +
      // poseEstimator.getEstimatedPosition().getRotation());
      // System.out.println("PRediodicing2");
      Logger.recordOutput("Drive/OdoYawFromGyro", rawGyroRotation.getDegrees());
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      Logger.recordOutput(
          "Drive/OdoYawRightAfter",
          poseEstimator.getEstimatedPosition().getRotation().getDegrees());

      // if (m_LimeLight1.measurmentValid()) {
      //   poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, 9999999));
      //   poseEstimator.addVisionMeasurement(
      //       m_LimeLight1.getMeasuremPosition(), m_LimeLight1.getMeasurementTimeStamp());
      // } // THE SDEVS ARE TOO HIGH (I THINK) causes jitter wehn seeing two measurments
      // else if (m_LimeLight2.measurmentValid()) {
      //   poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, 9999999));//Switched
      // from 0.7 to 0.1 after have a great conversation with the lead programmer on 5188
      //   poseEstimator.addVisionMeasurement(
      //       m_LimeLight2.getMeasuremPosition(), m_LimeLight2.getMeasurementTimeStamp());
      // }

      Optional<Alliance> ally = DriverStation.getAlliance();
      try {
        if (ally.get() == DriverStation.Alliance.Red) {
          currentRadiusFromReef =
              SeanMathUtil.distance(
                  poseEstimator.getEstimatedPosition(),
                  new Pose2d(
                      AutoAlignDesitationDeterminer.transform2red(Constants.Field.Reef.reefCenter),
                      new Rotation2d(0.0)));
        } else {
          currentRadiusFromReef =
              SeanMathUtil.distance(
                  poseEstimator.getEstimatedPosition(),
                  new Pose2d(Constants.Field.Reef.reefCenter, new Rotation2d(0.0)));
        }
      } catch (Exception e) {

      }
      specialPoseEstimation = currentRadiusFromReef < 1.8;
      if (!m_LimeLight1.measurmentValid()) {
        specialPoseEstimation =
            false; // TODO: this line says weather we go into megatag 1 when close to the reef
      }
      Logger.recordOutput("Drive/DistanceFromReef", currentRadiusFromReef);
      Logger.recordOutput("Drive/InSpecialMode", specialPoseEstimation);
      Logger.recordOutput("Drive/Algea-Mode", AutoAlignDesitationDeterminer.seekingAlgea);
      Logger.recordOutput("Drive/ICoral-For-L1", coralIntededforL1);
      Logger.recordOutput(
          "Drive/ICoral-For-L1-AutopALign", AutoAlignDesitationDeterminer.placingAtL1);
      try {
        Logger.recordOutput("Drive/Rotation-Power-From-Vision", getCoralYaw());
      } catch (Exception e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(
              .1, .1,
              9999999)); // Was 0.7, limelight recommends 0.5, 5188 0.1, and Sonic squirels 0.9
      if (specialPoseEstimation) {
        m_LimeLight1.setMegatag(true);
        m_LimeLight2.setMegatag(true);
        m_LimeLight3.setMegatag(true);
        m_LimeLight4.setMegatag(true);
      } else {
        m_LimeLight1.setMegatag(false);
        m_LimeLight2.setMegatag(false);
        m_LimeLight3.setMegatag(false);
        m_LimeLight4.setMegatag(false);
      }

      // Logger.recordOutput("Drive/limelight3Distance",
      // m_LimeLight3.getMeasurement().avgTagDist());
      if (m_LimeLight1.measurmentValid()) {
        poseEstimator.addVisionMeasurement(
            m_LimeLight1.getMeasuremPosition(), m_LimeLight1.getMeasurementTimeStamp());
      }
      if (m_LimeLight2.measurmentValid() && !specialPoseEstimation) {
        poseEstimator.addVisionMeasurement(
            m_LimeLight2.getMeasuremPosition(), m_LimeLight2.getMeasurementTimeStamp());
      }

      if (DriverStation.isDisabled()) {
        autoElapsedTime = null;
      } else if (DriverStation.isTeleop()) {
        autoElapsedTime = null;
      } else if (DriverStation.isAutonomousEnabled() && autoElapsedTime == null) {
        autoElapsedTime = new Timer();
        autoElapsedTime.start();
      }

      boolean inAuto = autoElapsedTime != null;

      boolean ignoreSwerveLimelight = inAuto && !autoElapsedTime.hasElapsed(3);

      Logger.recordOutput("Drive/IgnoringSwerveLimelight", ignoreSwerveLimelight);

      if (ignoreSwerveLimelight) {
        // Returns early if there is a autoElapsedTime (not teleop or disabled) and it is not over 3
        // seconds
        return;
      }

      if (m_LimeLight3.measurmentValid() && !specialPoseEstimation) {
        poseEstimator.addVisionMeasurement(
            m_LimeLight3.getMeasuremPosition(), m_LimeLight3.getMeasurementTimeStamp());
      }
      if (m_LimeLight4.measurmentValid() && !specialPoseEstimation) {
        poseEstimator.addVisionMeasurement(
            m_LimeLight4.getMeasuremPosition(), m_LimeLight4.getMeasurementTimeStamp());
      }
    }
  }

  private Timer autoElapsedTime;

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    Logger.recordOutput("Drive/SetPoseInput", pose);

    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    Logger.recordOutput("Drive/SetPoseOutputEstimation", poseEstimator.getEstimatedPosition());
  }

  /** Resets the current odometry pose. */
  public void setPoseDummy(Pose2d pose) {
    // poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  private ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public double getCoralYaw() {
    // double closestCoralYaw = LimelightHelpers.getTX("limelight-coral");

    String dump = LimelightHelpers.getJSONDump("limelight-coral");
    ObjectMapper objectMapper = new ObjectMapper();
    JsonNode detectorResults;
    try {
      detectorResults = objectMapper.readTree(dump).path("Results").get("Detector");
    } catch (JsonProcessingException e) {
      e.printStackTrace();
      return 0;
    }
    double closestCoralTx = 0.0;
    double closestCoralTy = 100.0;

    try {
      if (detectorResults.isArray()) {
        for (JsonNode result : detectorResults) {
          if (result.path("classID").asInt() == 1
              && result.path("ty").asDouble() < closestCoralTy) {
            closestCoralTx = result.path("tx").asDouble();
            closestCoralTy = result.path("ty").asDouble();
          }
        }
      }
    } catch (NullPointerException e) {
      // e.printStackTrace();
      return 0;
    } catch (Exception e) {
      // e.printStackTrace();
      return 0;
    }

    return Math.abs(closestCoralTx) <= 0.15
        ? 0
        : Math.max(
            Math.min((3 + -closestCoralTx) / 100, 0.1),
            -0.1); // This is the yaw of the closest coral tag

    // LimelightResults results = LimelightHelpers.getLatestResults("limelight-coral");
    // // System.out.println(LimelightHelpers.getT2DArray("limelight-coral").length);
    // // System.out.println(results.valid + " | " + results.targets_Detector.length);
    // if (results.valid) {
    //   if (results.targets_Detector.length > 0) {
    //     for (LimelightTarget_Detector detection : results.targets_Detector) {
    //       System.out.println(
    //           detection.className
    //               + " | "
    //               + detection.confidence
    //               + " | "
    //               + detection.ta
    //               + " | "
    //               + detection.tx
    //               + " | "
    //               + detection.ty);
    //     }
    //   }
    // }

    // List<LimelightTarget_Fiducial> targets = results.targetingResults.targets_Fiducials;

    // for (LimelightTarget_Fiducial target : targets) {
    //   System.out.println("X Position (tx): " + target.tx);

    // }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public Twist2d getfieldVelocity() {
    ChassisSpeeds robotVelocities = getSpeeds();
    Twist2d robotVelocity =
        new Twist2d(
            robotVelocities.vxMetersPerSecond,
            robotVelocities.vyMetersPerSecond,
            robotVelocities.omegaRadiansPerSecond);
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRads).toArray();
  }

  public ArrayList<Rotation2d> getRawTurnEncoderPositions() {
    ArrayList<Rotation2d> out = new ArrayList<Rotation2d>();

    for (Module module : modules) {
      out.add(module.getRawTurnEncoderPosition());
    }

    return out;
  }

  /** Get the position of all drive wheels in radians. */
  public void rezeroModulesRelativeEncoders() {
    for (Module module : modules) {
      module.rezeroRelativeEncoder();
    }
  }
}
