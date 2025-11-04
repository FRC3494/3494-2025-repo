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

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.LEDs.LEDLightPattern;
import frc.robot.subsystems.LoggedMitocandria;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.util.Elastic;

// import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private LoggedMitocandria loggedMitocandria;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Make Elastic layout available to DS
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        LoggedPowerDistribution.getInstance(
            Constants.POWER_DISTRIBUTION_PANEL_CAN_ID, PowerDistribution.ModuleType.kRev);
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    // Logger.registerURCL(URCL.startExternal());

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    loggedMitocandria = new LoggedMitocandria();
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    loggedMitocandria.periodic();
    OI.update();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // robotContainer.drive.canReadTags = false;
    robotContainer.leds.setPattern(LEDLightPattern.DISABLED).schedule();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    ModuleIOSparkMax.setGearRatio(Constants.Drivetrain.L1_GEAR_RATIO);
    robotContainer.drive.rezeroModulesRelativeEncoders(); // re-zero on auto init
    robotContainer.leds.setPattern(LEDLightPattern.NONE).schedule();
    // robotContainer.drive.canReadTags = true;
    Elastic.selectTab("Autonomous");

    Constants.DRIVE_MODE = DriveMode.NORMAL;
    robotContainer.arm.updatePIDLimits();
    robotContainer.elevator.updatePIDLimits();
    robotContainer.intake.updateIdleMode();

    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // robotContainer.drive.canReadTags = true;
    ModuleIOSparkMax.setGearRatio(Constants.Drivetrain.L1_GEAR_RATIO);
    robotContainer.leds.setPattern(LEDLightPattern.NONE).schedule();
    // ! Uncomment for comp
    Elastic.selectTab("Teleoperated");

    Constants.DRIVE_MODE = robotContainer.driveModeChooser.get();
    robotContainer.arm.updatePIDLimits();
    robotContainer.arm.setBrakes(IdleMode.kCoast);
    robotContainer.elevator.updatePIDLimits();
    robotContainer.intake.updateIdleMode();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (!robotContainer.arm.groundIntaking && robotContainer.arm.bufferedCommand != null) {
      robotContainer.arm.bufferedCommand.schedule();
      robotContainer.arm.bufferedCommand = null;
    }

    DriveMode chooserDriveMode = robotContainer.driveModeChooser.get();
    if (chooserDriveMode != Constants.DRIVE_MODE) {
      Constants.DRIVE_MODE = chooserDriveMode;

      switch (Constants.DRIVE_MODE) {
        case DEMO, DEMO_AUTOALIGN -> {
          robotContainer.arm.defenseMode = false;
          robotContainer.groundIntake.hoverPosition = Constants.Presets.groundIntakeHover;
        }
        default -> {
          robotContainer.arm.defenseMode = true;
          robotContainer.groundIntake.hoverPosition = Constants.Presets.groundIntakeStore;
        }
      }

      robotContainer.arm.updatePIDLimits();
      robotContainer.elevator.updatePIDLimits();
      robotContainer.intake.updateIdleMode();
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
