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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.WheelOffsetCalculator;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Intake intake;
  private final Elevator elevator;
  private final Arm arm;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();
    arm.setDefaultCommand(new TeleopArm(arm));
    elevator.setDefaultCommand(new TeleopElevator(elevator));
    intake.setDefaultCommand(new TeleopIntake(intake));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Wheel Radius Calc", new WheelRadiusCharacterization(drive, Direction.COUNTER_CLOCKWISE));
    //INTAKE STUFF-----------------------
    NamedCommands.registerCommand(
            "Intake", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand(
            "Outtake", new AutoIntakePower(intake, -1));
    NamedCommands.registerCommand(
            "Stop Intake", new AutoIntakePower(intake, 0));
    //Superstructure Place STUFF-----------------------
    NamedCommands.registerCommand(
            "L2 Outtake", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                        arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
        })));
    NamedCommands.registerCommand(
        "L2 Algea", Commands.sequence(
            new InstantCommand(
                () -> {
                    elevator.setElevatorPosition(Constants.Presets.liftIntake);
                    arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
        })));
    NamedCommands.registerCommand(
        "L3 Outtake", Commands.sequence(
                new InstantCommand(
                    () -> {
                      elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                      arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
        })));
    NamedCommands.registerCommand(
        "L3 Algea", Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                  arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
        })) );
    //Superstrucutre Intake Stuff-----------------------
    NamedCommands.registerCommand(
            "Intake Pos", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armIntake, 0);
                    })));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Calculate Wheel Position", new WheelOffsetCalculator(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(), // used to be -
            () -> -controller.getRightX())); // used to be -
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.back().onTrue(Commands.runOnce(
        () -> {
           GyroIOPigeon2.pigeon.setYaw(0.0);
        }));
    controller.
        leftBumper()
        .or(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("ALIGNING-------------------------------------------");
                  // DriveCommands.autoAlign(drive).execute();
                  System.out.println(drive.getDefaultCommand());
                  // ------------

                  // -----------
                  drive.setDefaultCommand(
                      DriveCommands.autoAlign(drive, controller.leftBumper().getAsBoolean()));
                  System.out.println(drive.getDefaultCommand());

                  // ------------

        }));
    controller
        .leftBumper()
        .or(controller.rightBumper())
        .onFalse(
            Commands.runOnce(
                () -> {
                  System.out.println("Stopping-------------------------------------------");
                  drive.setDefaultCommand(
                      DriveCommands.joystickDrive(
                          drive,
                          () -> -controller.getLeftY(),
                          () -> -controller.getLeftX(), // used to be -
                          () -> -controller.getRightX()));
                }));
    controller
        .leftBumper()
        .or(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPoseDummy(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    //======== L3 ============
    controller
        .y()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> {
                      elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                      arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
                    })));
    controller
        .y()
        .onFalse(
            Commands.sequence(
                new InstantCommand(
                    () -> {
                      elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                      arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
                    })));
    //========== L2 ===============
    controller
        .x().onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
                    })));
    controller
        .x().onFalse(
            Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                        arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
                    })));
    //========= L1 ==============
    //========= Intake ==============
    controller
        .b()
            .onTrue(
                Commands.sequence(
                    new InstantCommand(
                        () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            arm.setTargetAngle(Constants.Presets.armCoral, 0);
                        })));
    controller
        .b()
            .onFalse(
                Commands.sequence(
                    new InstantCommand(
                        () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            arm.setTargetAngle(Constants.Presets.armIntake, 0);
                        })));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = autoChooser.get();

    System.out.println("Starting: " + command.getName());

    return command;
  }
}
