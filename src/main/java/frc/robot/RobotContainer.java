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

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.LEDs.LEDLightPattern;
import frc.robot.commands.AutoAlignDesitationDeterminer;
import frc.robot.commands.AutoAutoAlign;
import frc.robot.commands.AutoIntakeDeadline;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.AutoPickupCoral;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeGroundCoral;
import frc.robot.commands.MainDriveCommand;
import frc.robot.commands.TeleopClimber;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.WheelOffsetCalculator;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.enums.Direction;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.util.SeanMathUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  public final Elevator elevator;
  public final Arm arm;
  public final Climber climber;
  public final GroundIntake groundIntake;
  public final LEDs leds;

  private final RobotCommands robotCommands;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public static Joystick leftButtonBoard = new Joystick(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final AutoFactory autoFactory;
  private final Autos autos;

  public final LoggedDashboardChooser<DriveMode> driveModeChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();
    climber = new Climber();
    groundIntake = new GroundIntake();
    leds = new LEDs();

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

    // arm.setDefaultCommand(new TeleopArm(arm));// the intake command overrides this so for now
    // its
    // content is going in the intake command
    elevator.setDefaultCommand(new TeleopElevator(elevator));
    intake.setDefaultCommand(
        new TeleopIntake(intake, arm, leds, groundIntake, () -> drive.coralIntededforL1));
    // arm.setDefaultCommand(new TeleopIntake(intake, arm));
    climber.setDefaultCommand(new TeleopClimber(climber));

    robotCommands = new RobotCommands(elevator, arm, groundIntake, intake, drive);

    // Set up auto routines
    NamedCommands.registerCommand(
        "VisionCoralGrab", new AutoPickupCoral(drive, groundIntake, arm, elevator, intake, 2));
    NamedCommands.registerCommand(
        "VisionCoralGrabFast",
        new AutoPickupCoral(drive, groundIntake, arm, elevator, intake, 4, -0.4));
    NamedCommands.registerCommand(
        "IntakeGroundCoral", new IntakeGroundCoral(groundIntake, arm, elevator, intake));
    NamedCommands.registerCommand(
        "Wheel Radius Calc", new WheelRadiusCharacterization(drive, Direction.COUNTER_CLOCKWISE));

    NamedCommands.registerCommand(
        "Blue-Left-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.196, 5.058, new Rotation2d(Math.toRadians(180))));
            }));
    NamedCommands.registerCommand(
        "Blue-4-peice-test-set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.196, 5.500, new Rotation2d(Math.toRadians(180))));
            }));
    NamedCommands.registerCommand(
        "Blue-Middle-Front-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.194, 7.634, new Rotation2d(Math.toRadians(-90))));
            }));
    NamedCommands.registerCommand(
        "Red-Right-Middle-Front-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(10.35105, 7.6338, new Rotation2d(Math.toRadians(90))));
            }));
    NamedCommands.registerCommand(
        "Blue-Right-Middle-Front-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.194, 0.418, new Rotation2d(Math.toRadians(-90))));
            }));
    NamedCommands.registerCommand(
        "Blue-Right-Fast-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.196, 2.994, new Rotation2d(Math.toRadians(0))));
            }));
    NamedCommands.registerCommand(
        "Blue-Right-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.652, 2.954, new Rotation2d(Math.toRadians(90))));
            }));
    NamedCommands.registerCommand(
        "Blue-Middle-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(7.550, 4.062, new Rotation2d(Math.toRadians(90))));
            })); // was 180, not sure why
    NamedCommands.registerCommand(
        "Red-Middle-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(9.995, 3.990, new Rotation2d(Math.toRadians(-90))));
            })); // was 180, not sure why
    NamedCommands.registerCommand(
        "Red-Left-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(10.349, 2.994, new Rotation2d(Math.toRadians(0))));
            }));
    NamedCommands.registerCommand(
        "Red-Right-Fast-Set-Pose",
        new InstantCommand(
            () -> {
              drive.setPose(new Pose2d(10.349, 5.058, new Rotation2d(Math.toRadians(180))));
            }));
    // INTAKE STUFF-----------------------
    NamedCommands.registerCommand("Intake", new AutoIntakePower(intake, -1));
    NamedCommands.registerCommand("Outtake", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand("Outtake Fast", new AutoIntakePower(intake, 0.75));
    // NamedCommands.registerCommand("Outtake Slow", new AutoIntakePower(intake, 0.25));
    NamedCommands.registerCommand("Outtake Algea", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand("Outtake L1", new AutoIntakePower(intake, -0.3));
    NamedCommands.registerCommand("Intake Deadline", new AutoIntakeDeadline(intake));
    NamedCommands.registerCommand("Stop Intake", new AutoIntakePower(intake, 0));
    // Superstructure Place STUFF-----------------------
    NamedCommands.registerCommand(
        "FreeArm",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  groundIntake.setIntakePosition(Constants.Presets.groundIntakePop);
                }),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> {
                  arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                })));
    NamedCommands.registerCommand(
        "L1",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntake);
                  arm.setTargetAngle(Constants.Presets.armOuttakeL1, 0);
                })));
    NamedCommands.registerCommand(
        "L2 Outtake",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                  arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
                })));
    NamedCommands.registerCommand(
        "L2 Algea",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntake);
                  arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
                })));
    NamedCommands.registerCommand(
        "GroundIntakeDown",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
                })));
    NamedCommands.registerCommand(
        "ArmPop",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntake);
                  arm.setTargetAngle(
                      groundIntake.wanttoPOP
                          ? Constants.Presets.armGroundTransferWithPop
                          : Constants.Presets.armGroundTransfer,
                      0);
                })));
    // NamedCommands.registerCommand(
    //     "HOLDALGEA", Commands.sequence(
    //         new InstantCommand(
    //             () -> {
    //                 elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //                 arm.setTargetAngle(Constants.Presets.armAlgeaL2Auto, 0);
    //     })));
    NamedCommands.registerCommand(
        "L3 Outtake",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                  arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
                })));
    NamedCommands.registerCommand(
        "L3 Outtake Delayed",
        Commands.sequence(
            new WaitCommand(0.5),
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                  arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
                })));
    NamedCommands.registerCommand(
        "L3 Algea",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftAlgeaL3);
                  arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
                })));
    // Superstrucutre Intake Stuff-----------------------
    NamedCommands.registerCommand(
        "Intake Pos",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntakeAlt);
                  arm.setTargetAngle(Constants.Presets.armIntakeAlt, 0);
                })));
    NamedCommands.registerCommand(
        "Processor",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntake);
                  arm.setTargetAngle(Constants.Presets.armProcessor, 0);
                })));

    NamedCommands.registerCommand(
        "Algea Pos",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftIntake);
                  arm.setTargetAngle(Constants.Presets.armBargeYeet, 0); // was armCoral
                })));
    NamedCommands.registerCommand(
        "BargeStage0",
        Commands.sequence(
            new InstantCommand(
                () -> {
                  elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                }),
            new InstantCommand(
                () -> {
                  groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
                }),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> {
                  arm.setPIDlimits(-0.4, 0.4);
                }),
            new InstantCommand(
                () -> {
                  arm.setTargetAngle(Constants.Presets.armBargeStore, 0);
                }),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> {
                  arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
                })));
    NamedCommands.registerCommand(
        "GOTOPosSwing",
        new AutoAutoAlign(
            drive,
            1,
            Constants.Auto.AmpMidAuto.posSwing,
            4.5,
            Constants.Drivetrain.getMaxLinearAcceleration(),
            Constants.Drivetrain.getMaxAngularVelocity())); // todo
    NamedCommands.registerCommand(
        "GOTOPos1",
        new AutoAutoAlign(
            drive,
            1,
            Constants.Auto.AmpMidAuto.pos1,
            4.5,
            Constants.Drivetrain.getMaxLinearAcceleration(),
            Constants.Drivetrain.getMaxAngularVelocity())); // todo
    NamedCommands.registerCommand(
        "GOTOPos2",
        new AutoAutoAlign(
            drive,
            2,
            Constants.Auto.AmpMidAuto.pos2,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.5,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    NamedCommands.registerCommand(
        "GOTOPos2pt2",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.AmpMidAuto.pos2,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.5,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    NamedCommands.registerCommand(
        "GOTOPluck", new AutoAutoAlign(drive, 0.25, Constants.Auto.AmpMidAuto.algeaPluck)); // todo
    NamedCommands.registerCommand(
        "GOTOPos3", new AutoAutoAlign(drive, 1.2, Constants.Auto.AmpMidAuto.pos3)); // todo
    NamedCommands.registerCommand(
        "GOTOPos4", new AutoAutoAlign(drive, 1.2, Constants.Auto.AmpMidAuto.pos4)); // todo
    NamedCommands.registerCommand(
        "GOTOPos5",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.AmpMidAuto.pos5,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.3,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    // RED COMMANDS STUFFFFF
    NamedCommands.registerCommand(
        "GOTOPosSwingRed",
        new AutoAutoAlign(
            drive,
            1,
            Constants.Auto.AmpMidAutoRed.posSwing,
            4.5,
            Constants.Drivetrain.getMaxLinearAcceleration(),
            Constants.Drivetrain.getMaxAngularVelocity())); // todo
    NamedCommands.registerCommand(
        "GOTOPos1Red",
        new AutoAutoAlign(
            drive,
            1,
            Constants.Auto.AmpMidAutoRed.pos1,
            4.5,
            Constants.Drivetrain.getMaxLinearAcceleration(),
            4.8)); // todo
    NamedCommands.registerCommand(
        "GOTOPos2Red",
        new AutoAutoAlign(
            drive,
            2,
            Constants.Auto.AmpMidAutoRed.pos2,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.5,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    NamedCommands.registerCommand(
        "GOTOPos2pt2Red",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.AmpMidAutoRed.pos2,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.5,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    NamedCommands.registerCommand(
        "GOTOPluckRed",
        new AutoAutoAlign(drive, 0.25, Constants.Auto.AmpMidAutoRed.algeaPluck)); // todo
    NamedCommands.registerCommand(
        "GOTOPos3Red", new AutoAutoAlign(drive, 1.2, Constants.Auto.AmpMidAutoRed.pos3)); // todo
    NamedCommands.registerCommand(
        "GOTOPos4Red", new AutoAutoAlign(drive, 1.2, Constants.Auto.AmpMidAutoRed.pos4)); // todo
    NamedCommands.registerCommand(
        "GOTOPos5Red",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.AmpMidAutoRed.pos5,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.3,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.3)); // todo
    // END OF RED COMMANDS STUFF

    // START OF $PEICE TEST STUFF
    NamedCommands.registerCommand(
        "4-GOTOPos1",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.ThreePeiceCoolAuto.pos1,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    NamedCommands.registerCommand(
        "4-GOTOPos2",
        new AutoAutoAlign(
            drive,
            0.5,
            Constants.Auto.ThreePeiceCoolAuto.pos2,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    NamedCommands.registerCommand(
        "4-GOTOPos3",
        new AutoAutoAlign(
            drive,
            3,
            Constants.Auto.ThreePeiceCoolAuto.pos3,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    NamedCommands.registerCommand(
        "4-GOTOPos4",
        new AutoAutoAlign(
            drive,
            0.5,
            Constants.Auto.ThreePeiceCoolAuto.pos4,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    NamedCommands.registerCommand(
        "4-GOTOPos5",
        new AutoAutoAlign(
            drive,
            3,
            Constants.Auto.ThreePeiceCoolAuto.pos5,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    NamedCommands.registerCommand(
        "4-GOTOPos7",
        new AutoAutoAlign(
            drive,
            2.5,
            Constants.Auto.ThreePeiceCoolAuto.pos7,
            3.5,
            Constants.Drivetrain.getMaxLinearAcceleration() * 0.4,
            Constants.Drivetrain.getMaxAngularVelocity() * 0.6));
    //
    NamedCommands.registerCommand( // THIS IS IN AUTO, IF YOU WANNA TUNE DONT RUN THIS ONE
        "Barge", robotCommands.barge());

    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );
    autos = new Autos(autoFactory, intake, elevator, arm, groundIntake, drive, robotCommands);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("=====================", Commands.none());

    autoChooser.addOption("Wraparound", autos.AutoRoutines.wraparound());
    autoChooser.addOption("Backside", autos.AutoRoutines.backside());
    autoChooser.addOption("Barge", autos.AutoRoutines.barge());

    autoChooser.addOption("====================", Commands.none());

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
    autoChooser.addOption(
        "Wheel Radius Calc", new WheelRadiusCharacterization(drive, Direction.COUNTER_CLOCKWISE));
    autoChooser.addOption("Outtake Test", new AutoIntakePower(intake, -1));

    driveModeChooser = new LoggedDashboardChooser<>("Teleop DriveMode");
    driveModeChooser.addDefaultOption("NORMAL", DriveMode.DEMO);
    for (DriveMode mode : DriveMode.values()) {
      driveModeChooser.addOption(mode.toString(), mode);
    }

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
            () -> -Constants.Drivetrain.rotationPower(controller.getRightX()))); // used to be -
    // controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .back()
        .and(controller.start().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    GyroIOPigeon2.pigeon.setYaw(180.0);
                  } else {
                    GyroIOPigeon2.pigeon.setYaw(0.0);
                  }
                }));
    controller
        .start()
        .and(controller.back())
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.rezeroModulesRelativeEncoders();
                }));
    OI.rezeroElevator()
        .rising()
        .ifHigh(
            () -> {
              elevator.rezeroElevator();
              ;
            });

    OI.ToggleDefenseMode()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        if (arm.defenseMode == false) {
                          arm.defenseMode = true;
                          groundIntake.hoverPosition = Constants.Presets.groundIntakeStore;
                          Constants.Presets.defenseDelay = 0.5;
                        } else {
                          arm.defenseMode = false;
                          groundIntake.hoverPosition = Constants.Presets.groundIntakeHover;
                          Constants.Presets.defenseDelay = 0.0;
                        }
                      })
                  .ignoringDisable(false)
                  .schedule();
            });
    OI.toggleDistanceSensor()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        groundIntake.wanttoPOP = !groundIntake.wanttoPOP;
                      })
                  .ignoringDisable(false)
                  .schedule();
            });

    // Coral Align
    controller
        .leftBumper()
        .and(controller.rightBumper().negate())
        .or(controller.rightBumper().and(controller.leftBumper().negate()))
        .and(Constants::getAutoAlignEnabled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("ALIGNING CORAL-------------------------------------------");
                  System.out.println(drive.getDefaultCommand());
                  drive.setDefaultCommand(
                      DriveCommands.autoAlign(
                          drive, controller.leftBumper().getAsBoolean(), false, false));
                  System.out.println(drive.getDefaultCommand());
                }));

    // Algae Align
    controller
        .leftBumper()
        .and(controller.rightBumper())
        .and(Constants::getAutoAlignEnabled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("ALIGNING ALGAE-------------------------------------------");
                  System.out.println(drive.getDefaultCommand());
                  drive.setDefaultCommand(DriveCommands.autoAlign(drive, false, false, true));
                  System.out.println(drive.getDefaultCommand());
                }));

    // Barge Align
    controller
        .povLeft()
        .and(Constants::getAutoAlignEnabled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("ALIGNING BARGE-------------------------------------------");
                  System.out.println(drive.getDefaultCommand());
                  drive.setDefaultCommand(
                      DriveCommands.autoAlign(
                          drive,
                          false,
                          true,
                          controller.leftBumper().getAsBoolean()
                              && controller.rightBumper().getAsBoolean()));
                  System.out.println(drive.getDefaultCommand());
                }));

    controller
        .leftBumper()
        .or(controller.rightBumper())
        .or(controller.povLeft())
        .and(Constants::getAutoAlignEnabled)
        .onFalse(
            Commands.runOnce(
                () -> {
                  System.out.println("Stopping-------------------------------------------");
                  drive.setDefaultCommand(
                      DriveCommands.joystickDrive(
                          drive,
                          () -> -controller.getLeftY(),
                          () -> -controller.getLeftX(),
                          () -> -controller.getRightX()));
                }));

    controller
        .leftBumper()
        .or(controller.rightBumper())
        .or(controller.x())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPoseDummy(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  MainDriveCommand.coralAligning = true;
                }));
    controller
        .povRight()
        .onFalse(
            new InstantCommand(
                () -> {
                  MainDriveCommand.coralAligning = false;
                }));

    controller.y().onTrue(drive.toggleFastRotation());

    // ======== L3 ============
    OI.L3Algea()
        .rising()
        .ifHigh(
            () -> {
              Command l3AlgeaCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftAlgeaL3);
                            arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return SeanMathUtil.compareArmPosition(
                                arm.getPosition(), Constants.Presets.armAlgeaL3, 0.1);
                          }));
              if (!arm.groundIntaking) {
                l3AlgeaCommand.schedule();
              } else {
                arm.bufferedCommand = l3AlgeaCommand;
              }
            });
    OI.L3Algea()
        .falling()
        .ifHigh(
            () -> {
              Command l3AlgeaCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                            arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
                          }));
              if (!arm.groundIntaking) {
                l3AlgeaCommand.schedule();
              } else {
                arm.bufferedCommand = l3AlgeaCommand;
              }
            });
    OI.L3Coral()
        .rising()
        .ifHigh(
            () -> {
              Command l3CoralCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                            arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return SeanMathUtil.compareArmPosition(
                                arm.getPosition(), Constants.Presets.armOuttakeL3, 0.1);
                          }));
              if (!arm.groundIntaking) {
                l3CoralCommand.schedule();
              } else {
                arm.bufferedCommand = l3CoralCommand;
              }
            });
    // ========== L2 ===============
    OI.L2Algea()
        .rising()
        .ifHigh(
            () -> {
              Command l2AlgeaCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftAlgeaTeleopL2);
                            arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return SeanMathUtil.compareArmPosition(
                                arm.getPosition(), Constants.Presets.armAlgeaL2, 0.1);
                          }));
              if (!arm.groundIntaking) {
                l2AlgeaCommand.schedule();
              } else {
                arm.bufferedCommand = l2AlgeaCommand;
              }
            });
    OI.L2Algea()
        .falling()
        .ifHigh(
            () -> {
              Command l2AlgeaCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
                          }));
              if (!arm.groundIntaking) {
                l2AlgeaCommand.schedule();
              } else {
                arm.bufferedCommand = l2AlgeaCommand;
              }
            });
    OI.L2Coral()
        .rising()
        .ifHigh(
            () -> {
              Command l2CoralCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return SeanMathUtil.compareArmPosition(
                                arm.getPosition(), Constants.Presets.armOuttakeL2, 0.1);
                          }));
              if (!arm.groundIntaking) {
                l2CoralCommand.schedule();
              } else {
                arm.bufferedCommand = l2CoralCommand;
              }
            });
    // controller
    //     .x().or(()->leftButtonBoard.getRawButton(4))
    //     .onTrue(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //                     arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
    //                 })));
    // controller
    //     .x().or(()->leftButtonBoard.getRawButton(5)).or(() ->
    // leftButtonBoard.getRawButtonReleased(4))
    //     .onFalse(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                     elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
    //                     arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
    //                 })));
    // ========= L1 ==============
    // controller
    //     .a().or(()->leftButtonBoard.getRawButton(9))
    //     .onFalse(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //                     arm.setTargetAngle(Constants.Presets.armOuttakeL1, 0);
    //                 })));
    OI.armSafeMode()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                      })
                  .schedule();
            });
    OI.l1Test()
        .rising()
        .ifHigh(
            () -> {
              Command l1TestCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.L1elevatorTest);
                            arm.setTargetAngle(Constants.Presets.L1armtest, 0);
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return SeanMathUtil.compareArmPosition(
                                arm.getPosition(), Constants.Presets.L1armtest, 0.1);
                          }));
              if (!arm.groundIntaking) {
                l1TestCommand.schedule();
              } else {
                arm.bufferedCommand = l1TestCommand;
              }
            });
    // ========= Intake ==============
    OI.feeder()
        .rising()
        .ifHigh(
            () -> {
              Command intakeCommand =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                          }),
                      new WaitCommand(0.25),
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntakeAlt);
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeStation);
                            groundIntake.setIntakePower(0, 0);
                          }),
                      new WaitCommand(0.5),
                      new InstantCommand(
                          () -> {
                            arm.setTargetAngle(Constants.Presets.armIntakeAlt, 0);
                            arm.groundIntaking = false;
                            drive.coralIntededforL1 = false;
                            AutoAlignDesitationDeterminer.placingAtL1 = false;
                          }));
              if (!arm.groundIntaking) {
                intakeCommand.schedule();
              } else {
                arm.bufferedCommand = intakeCommand;
              }
            });
    // OI.Intake().rising().ifHigh(()->{
    //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //     arm.setTargetAngle(Constants.Presets.armIntakeLow, 0);
    // });
    OI.Processor()
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            arm.setTargetAngle(Constants.Presets.armProcessor, 0);
                          }),
                      new WaitUntilCommand(
                          () ->
                              SeanMathUtil.compareArmPosition(
                                  arm.getPosition(), Constants.Arm.safePosition, false)),
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeStore);
                          }))
                  .ignoringDisable(false)
                  .schedule();
            });
    OI.lolipop()
        .rising()
        .ifHigh(
            () -> {
              Command lolipop =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            arm.setTargetAngle(Constants.Presets.armLoliPop, 0);
                          }));
              if (!arm.groundIntaking) {
                lolipop.schedule();
              } else {
                arm.bufferedCommand = lolipop;
              }
            });

    OI.activateGroundIntake()
        .rising()
        .ifHigh(
            () -> {
              System.out.println(
                  "Ground Intake Activated ---------------------------------------------------------------------");
              if (arm.getTargetPosition()
                  == Constants.Presets.armIntakeAlt + Constants.Presets.globalArmOffset) {
                Constants.Presets.defenseDelay = 1;
              } else {
                Constants.Presets.defenseDelay = 0;
              }
              if (groundIntake.targetPosition == Constants.Presets.groundIntakeL1
                  || groundIntake.targetPosition == Constants.Presets.groundIntakeStation
                  || groundIntake.targetPosition == Constants.Presets.groundIntakeStore) {
                Constants.Presets.defenseDelay = 1;
              } else {
                Constants.Presets.defenseDelay = 0;
              }
              System.out.println(drive.coralIntededforL1);
              if (drive.coralIntededforL1) {
                Commands.sequence(
                        leds.setPattern(LEDLightPattern.INTAKING),
                        new InstantCommand(
                            () -> {
                              arm.groundIntaking = true;
                              if (arm.getTargetPosition()
                                  == Constants.Presets.armIntakeAlt
                                      + Constants.Presets.globalArmOffset) {
                                elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                                // Constants.Presets.defenseDelay = 2;
                              } else {
                                elevator.setElevatorPosition(Constants.Presets.liftIntake);
                                // Constants.Presets.defenseDelay = 0;
                              }

                              arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                              groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
                            }),
                        new WaitUntilCommand(
                            () -> {
                              return SeanMathUtil.comparePosition(
                                  groundIntake.getPivotPosition(),
                                  Constants.Presets.groundIntakeIntake,
                                  0.15);
                            }),
                        new InstantCommand(
                            () -> {
                              elevator.setElevatorPosition(Constants.Presets.liftIntake);
                              arm.setTargetAngle(
                                  groundIntake.wanttoPOP
                                      ? Constants.Presets.armGroundTransferWithPop
                                      : Constants.Presets.armGroundTransfer,
                                  0);
                              drive.coralIntededforL1 = false;
                              AutoAlignDesitationDeterminer.placingAtL1 = false;
                            }),
                        new WaitUntilCommand(
                            () -> {
                              return SeanMathUtil.compareArmPosition(
                                      arm.getPosition(),
                                      groundIntake.wanttoPOP
                                          ? Constants.Presets.armGroundTransferWithPop
                                          : Constants.Presets.armGroundTransfer,
                                      0.05)
                                  && SeanMathUtil.comparePosition(
                                      elevator.getTicks(), Constants.Presets.liftIntake, 5);
                            }),
                        new InstantCommand(
                            () -> {
                              groundIntake.setIntakePower(-0.85, 0.85);
                            }))
                    .schedule();
              } else {
                Commands.sequence(
                        leds.setPattern(LEDLightPattern.INTAKING),
                        new InstantCommand(
                            () -> {
                              arm.groundIntaking = true;
                              if (arm.getTargetPosition()
                                  == Constants.Presets.armIntakeAlt
                                      + Constants.Presets.globalArmOffset) {
                                elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                                // Constants.Presets.defenseDelay = 2;
                              } else {
                                elevator.setElevatorPosition(Constants.Presets.liftIntake);
                                // Constants.Presets.defenseDelay = 0;
                              }

                              arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                              groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
                            }),
                        new WaitUntilCommand(
                            () -> {
                              return SeanMathUtil.comparePosition(
                                  groundIntake.getPivotPosition(),
                                  Constants.Presets.groundIntakeIntake,
                                  0.15);
                            }),
                        new InstantCommand(
                            () -> {
                              elevator.setElevatorPosition(Constants.Presets.liftIntake);
                              arm.setTargetAngle(
                                  groundIntake.wanttoPOP
                                      ? Constants.Presets.armGroundTransferWithPop
                                      : Constants.Presets.armGroundTransfer,
                                  0);
                              drive.coralIntededforL1 = false;
                              AutoAlignDesitationDeterminer.placingAtL1 = false;
                            }),
                        new WaitUntilCommand(
                            () -> {
                              return SeanMathUtil.compareArmPosition(
                                      arm.getPosition(),
                                      groundIntake.wanttoPOP
                                          ? Constants.Presets.armGroundTransferWithPop
                                          : Constants.Presets.armGroundTransfer,
                                      0.05)
                                  && SeanMathUtil.comparePosition(
                                      elevator.getTicks(), Constants.Presets.liftIntake, 5);
                            }),
                        new InstantCommand(
                            () -> {
                              groundIntake.setIntakePower(-0.85, 0.85);
                            }))
                    .schedule();
              }
              Commands.sequence(
                      new WaitUntilCommand(
                          () -> {
                            return groundIntake.getDistanceSensorTripped();
                          }),
                      new InstantCommand(
                          () -> {
                            if (OI.activateGroundIntake().getAsBoolean()) {
                              leds.setPattern(LEDLightPattern.HAS_GAMEPIECE).schedule();
                            }
                          }))
                  .schedule();
            });

    OI.activateGroundIntake()
        .falling()
        .ifHigh(
            () -> {
              System.out.println(
                  "Ground Intake Deactivated ---------------------------------------------------------------------");
              Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            groundIntake.setIntakePower(0, 0);
                            if (Constants.DRIVE_MODE != DriveMode.DEMO
                                && Constants.DRIVE_MODE != DriveMode.DEMO_AUTOALIGN) {
                              arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                            }
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return Constants.DRIVE_MODE == DriveMode.DEMO
                                || Constants.DRIVE_MODE == DriveMode.DEMO_AUTOALIGN
                                || SeanMathUtil.compareArmPosition(
                                    arm.getPosition(), Constants.Presets.armSafePosition, 0.07);
                          }),
                      new InstantCommand(
                          () -> {
                            System.out.println(groundIntake.hoverPosition);
                            groundIntake.setIntakePosition(
                                (Constants.DRIVE_MODE == DriveMode.DEMO
                                        || Constants.DRIVE_MODE == DriveMode.DEMO_AUTOALIGN)
                                    ? Constants.Presets.groundIntakeHover
                                    : groundIntake.hoverPosition);
                            arm.groundIntaking = false;
                          }),
                      new InstantCommand(
                          () -> {
                            if (leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE) {
                              leds.setPattern(LEDLightPattern.NONE).schedule();
                            }
                          }))
                  .schedule();
            });

    OI.L1GroundIntake()
        .rising()
        .ifHigh(
            () -> {
              Command l1gIntake =
                  Commands.sequence(
                      leds.setPattern(LEDLightPattern.INTAKING),
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftIntake);
                            arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
                            groundIntake.setIntakePower(-0.85, -0.6); // -0.85, -0.6
                            drive.coralIntededforL1 = true;
                            AutoAlignDesitationDeterminer.placingAtL1 = true;
                          }),
                      new WaitUntilCommand(
                          () -> {
                            return groundIntake.getDistanceSensorTripped();
                          }),
                      new InstantCommand(
                          () -> {
                            if (OI.L1GroundIntake().getAsBoolean()) {
                              leds.setPattern(LEDLightPattern.HAS_GAMEPIECE).schedule();
                            }
                          }));
              if (!arm.groundIntaking) {
                l1gIntake.schedule();
              } else {
                arm.bufferedCommand = l1gIntake;
              }
            });
    OI.L1GroundIntake()
        .falling()
        .ifHigh(
            () -> {
              Command l1gItnake =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePosition(
                                (Constants.DRIVE_MODE == DriveMode.DEMO
                                        || Constants.DRIVE_MODE == DriveMode.DEMO_AUTOALIGN)
                                    ? Constants.Presets.groundIntakeHover
                                    : Constants.Presets.groundIntakeL1);
                            groundIntake.setIntakePower(0, 0);
                          }),
                      new InstantCommand(
                          () -> {
                            if (leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE) {
                              leds.setPattern(LEDLightPattern.NONE).schedule();
                            }
                          }));

              if (!arm.groundIntaking) {
                l1gItnake.schedule();
              } else {
                arm.bufferedCommand = l1gItnake;
              }
            });
    OI.L1GroundIntake()
        .falling()
        .ifHigh(
            () -> {
              Command l1GroundIntakeHigh =
                  Commands.sequence(
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePosition(
                                (Constants.DRIVE_MODE == DriveMode.DEMO
                                        || Constants.DRIVE_MODE == DriveMode.DEMO_AUTOALIGN)
                                    ? Constants.Presets.groundIntakeHover
                                    : Constants.Presets.groundIntakeL1High);
                            groundIntake.setIntakePower(0, 0);
                          }),
                      new InstantCommand(
                          () -> {
                            if (leds.getPattern() != LEDLightPattern.HAS_GAMEPIECE) {
                              leds.setPattern(LEDLightPattern.NONE).schedule();
                            }
                          }));

              if (!arm.groundIntaking) {
                l1GroundIntakeHigh.schedule();
              } else {
                arm.bufferedCommand = l1GroundIntakeHigh;
              }
            });

    OI.groundIntakeOuttake()
        .or(
            () -> {
              return (drive.coralIntededforL1
                  ? (OI.controllerOuttake() > Constants.Intake.DEADBAND)
                  : false);
            })
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      leds.setPattern(LEDLightPattern.DEPOSITED),
                      new WaitUntilCommand(
                          () ->
                              !SeanMathUtil.compareArmPosition(
                                  arm.getPosition(),
                                  groundIntake.wanttoPOP
                                      ? Constants.Presets.armGroundTransferWithPop
                                      : Constants.Presets.armGroundTransfer,
                                  0.1)),
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeL1);
                            groundIntake.setIntakeCUrrentlim(60);
                          }),
                      new WaitUntilCommand(
                          () ->
                              SeanMathUtil.comparePosition(
                                  groundIntake.getPivotPosition(),
                                  Constants.Presets.groundIntakeL1,
                                  0.05)),
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePower(0.25, -0.25);
                          }),
                      new WaitCommand(0.0),
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeJerk);
                          }),
                      new WaitCommand(0.3),
                      new InstantCommand(
                          () -> {
                            groundIntake.setIntakeCUrrentlim(45);
                          }))
                  .schedule();
            });
    OI.groundIntakeOuttake()
        .or(
            () -> {
              return (drive.coralIntededforL1
                  ? (OI.controllerOuttake() > Constants.Intake.DEADBAND)
                  : false);
            })
        .falling()
        .ifHigh(
            () -> {
              groundIntake.setIntakePower(0, 0);
              leds.setPattern(LEDLightPattern.NONE).schedule();
            });
    OI.groundIntakeIntake()
        .rising()
        .ifHigh(
            () -> {
              groundIntake.setIntakePower(-0.2, 0.5);
            });
    OI.groundIntakeIntake()
        .falling()
        .ifHigh(
            () -> {
              groundIntake.setIntakePower(0, 0);
            });
    OI.groundIntakeManualOut()
        .rising()
        .ifHigh(
            () -> {
              groundIntake.setIntakePower(1.0, 0);
              leds.setPattern(LEDLightPattern.NONE).schedule();
            });
    OI.groundIntakeManualOut()
        .falling()
        .ifHigh(
            () -> {
              groundIntake.setIntakePower(0, 0);
            });
    OI.groundHover()
        .rising()
        .ifHigh(
            () -> {
              groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
            });
    OI.groundIntakeUp()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        if (!SeanMathUtil.comparePosition(
                            arm.getPosition(),
                            groundIntake.wanttoPOP
                                ? Constants.Presets.armGroundTransferWithPop
                                : Constants.Presets.armGroundTransfer,
                            0.1)) {
                          groundIntake.setIntakePosition(Constants.Presets.groundIntakeJerk);
                        }
                      })
                  .schedule();
            });
    // LOW INTAKE======================
    // OI.lowIntake().falling().ifHigh(()->{
    //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //     arm.setTargetAngle(Constants.Presets.armIntakeLow, 0);
    // });
    // OI.lowIntake().rising().ifHigh(()->{
    //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //     arm.setTargetAngle(Constants.Presets.armIntakeLowLow, 0);
    // });
    // OI.lowLowIntake().rising().ifHigh(()->{
    //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //     arm.setTargetAngle(Constants.Presets.armIntakeLowLow, 0);
    // });
    // BARGE===================
    OI.bargeStage()
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                          }),
                      new WaitCommand(0.5),
                      new InstantCommand(
                          () -> {
                            arm.setTargetAngle(Constants.Presets.armBargeStore, 0);
                          }))
                  .schedule();
            });
    OI.bargeYeet()
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      robotCommands.barge(),
                      new WaitCommand(1),
                      new InstantCommand(
                          () -> {
                            intake.setSpeed(0);
                          }),
                      robotCommands.store())
                  .schedule();
            });

    // OI.ClimbStage0().rising().ifHigh(()->{
    //     Commands.sequence(
    //         new PrintCommand("UNOVERCENTERING------------------"),
    //         new InstantCommand(()->{climber.setMotorPower(0.3);}),
    //         new WaitCommand(0.7),
    //         new InstantCommand(()->{climber.setMotorPower(-0.4);}),
    //         new WaitCommand(0.4),
    //         new InstantCommand(()->{climber.setMotorPower(0);})
    //     ).schedule();
    //     //climber.setTargetAngle(Constants.Presets.climberStage0, 0);
    // });

    OI.startClimb()
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      new InstantCommand(
                          () -> {
                            elevator.setElevatorPosition(Constants.Presets.liftClimb);
                            arm.setTargetAngle(Constants.Presets.armClimb, 0);
                            groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
                          }),
                      new WaitUntilCommand(
                          () -> elevator.getTicks() >= (Constants.Presets.liftClimb - 2.0)),
                      new InstantCommand(
                          () -> {
                            climber.setTargetAngle(0, 0);
                          }))
                  .ignoringDisable(false)
                  .schedule();
            });

    OI.ClimbStage1()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        climber.setCurrentLimit(20);
                        climber.setTargetAngle(Constants.Presets.climberStage1, 0);
                      })
                  .ignoringDisable(false)
                  .schedule();
            });

    OI.ClimbStage2()
        .rising()
        .ifHigh(
            () -> {
              Commands.sequence(
                      new InstantCommand(
                          () -> {
                            climber.setMotorBreak();
                          }),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(80);
                          }),
                      new InstantCommand(
                          () -> {
                            climber.setTargetAngle(Constants.Presets.climberStage2, 0);
                          }),
                      new WaitCommand(1.5),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(70);
                          }),
                      new WaitCommand(0.1),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(60);
                          }),
                      new WaitCommand(0.1),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(50);
                          }),
                      new WaitCommand(0.1),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(40);
                          }),
                      new WaitCommand(0.1),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(20);
                          }),
                      new WaitCommand(0.1),
                      new InstantCommand(
                          () -> {
                            climber.setCurrentLimit(0);
                          }))
                  .ignoringDisable(false)
                  .schedule();
            });

    OI.climberHover()
        .rising()
        .ifHigh(
            () -> {
              new InstantCommand(
                      () -> {
                        climber.setTargetAngle(Constants.Presets.climberHover, 0);
                      })
                  .ignoringDisable(false)
                  .schedule();
            });
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
