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

import java.net.ContentHandler;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignDesitationDeterminer;
import frc.robot.commands.AutoIntakeDeadline;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.BargFligIntake;
import frc.robot.commands.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopClimber;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.WheelOffsetCalculator;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.GroundIntake;
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
  public  final Elevator elevator;
  public final Arm arm;
  public final Climber climber;
  public final GroundIntake groundIntake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public static Joystick leftButtonBoard = new Joystick(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();
    climber = new Climber();
    groundIntake = new GroundIntake();
    //arm.setDefaultCommand(new TeleopArm(arm)); the intake command overrides this so for now its content is going in the intake command
    elevator.setDefaultCommand(new TeleopElevator(elevator));
    intake.setDefaultCommand(new TeleopIntake(intake, arm));
    // arm.setDefaultCommand(new TeleopIntake(intake, arm));
    climber.setDefaultCommand(new TeleopClimber(climber));

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
    NamedCommands.registerCommand(
        "Blue-Left-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(7.196, 5.058, new Rotation2d(Math.toRadians(180))));}));
    NamedCommands.registerCommand(
        "Blue-Right-Fast-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(7.196, 2.994, new Rotation2d(Math.toRadians(0))));}));
    NamedCommands.registerCommand(
        "Blue-Right-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(7.652, 2.954, new Rotation2d(Math.toRadians(90))));}));
    NamedCommands.registerCommand(
        "Blue-Middle-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(7.550, 4.062, new Rotation2d(Math.toRadians(90))));}));//was 180, not sure why
    NamedCommands.registerCommand(
        "Red-Middle-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(9.995, 3.990, new Rotation2d(Math.toRadians(-90))));}));//was 180, not sure why
    NamedCommands.registerCommand(
        "Red-Left-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(10.349, 2.994, new Rotation2d(Math.toRadians(0))));}));
    NamedCommands.registerCommand(
        "Red-Right-Fast-Set-Pose",  new InstantCommand(
            () -> {drive.setPose(new Pose2d(10.349, 5.058, new Rotation2d(Math.toRadians(180))));}));
    //INTAKE STUFF-----------------------
    NamedCommands.registerCommand(
            "Intake", new AutoIntakePower(intake, -1));
    NamedCommands.registerCommand(
            "Outtake", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand(
                "Outtake Fast", new AutoIntakePower(intake, 0.75));
    NamedCommands.registerCommand(
                "Outtake Algea", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand(
            "Outtake L1", new AutoIntakePower(intake, -0.3));
    NamedCommands.registerCommand(
            "Intake Deadline", new AutoIntakeDeadline(intake));
    NamedCommands.registerCommand(
            "Stop Intake", new AutoIntakePower(intake, 0));
    //Superstructure Place STUFF-----------------------
    NamedCommands.registerCommand(
            "FreeArm", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                    }),
                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                    groundIntake.setIntakePosition(Constants.Presets.groundIntakeStation);
                })
                    ));
    NamedCommands.registerCommand(
            "L1", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armOuttakeL1, 0);
                    })));
    NamedCommands.registerCommand(
            "L2 Outtake", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                        arm.setTargetAngle(Constants.Presets.armOuttakeL2Auto, 0);
        })));
    NamedCommands.registerCommand(
        "L2 Algea", Commands.sequence(
            new InstantCommand(
                () -> {
                    elevator.setElevatorPosition(Constants.Presets.liftIntake);
                    arm.setTargetAngle(Constants.Presets.armAlgeaL2Auto, 0);
        })));
    // NamedCommands.registerCommand(
    //     "HOLDALGEA", Commands.sequence(
    //         new InstantCommand(
    //             () -> {
    //                 elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //                 arm.setTargetAngle(Constants.Presets.armAlgeaL2Auto, 0);
    //     })));
    NamedCommands.registerCommand(
        "L3 Outtake", Commands.sequence(
                new InstantCommand(
                    () -> {
                      elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
                      arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
        })));
    NamedCommands.registerCommand(
        "L3 Outtake Delayed", Commands.sequence(
                new WaitCommand(0.5),
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
                        elevator.setElevatorPosition(Constants.Presets.liftIntakeAlt);
                        arm.setTargetAngle(Constants.Presets.armIntakeAlt, 0);
                    })));
    NamedCommands.registerCommand(
            "Processor", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armCoral, 0);
                    })));
                    
    NamedCommands.registerCommand(
            "Algea Pos", Commands.sequence(
                new InstantCommand(
                    () -> {
                        elevator.setElevatorPosition(Constants.Presets.liftIntake);
                        arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);//was armCoral
                    })));
    NamedCommands.registerCommand(
            "BargeStage0", Commands.sequence(
                new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
                new InstantCommand(() -> {groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);}),
                new WaitCommand(0.5),
                new InstantCommand(()->{arm.setPIDlimits(-0.4, 0.4);}),
                new InstantCommand(()-> {arm.setTargetAngle(Constants.Presets.armBargeStore, 0);}),
                new WaitCommand(1.5),
                new InstantCommand(()->{arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);})
            ));
                    
    NamedCommands.registerCommand(//THIS IS IN AUTO, IF YOU WANNA TUNE DONT RUN THIS ONE
            "Barge",Commands.sequence(
                new InstantCommand(() -> {arm.setCurrentLimit(85);}),
                // new InstantCommand(() -> {elevator.setPIDlimits(-1, 1);}),
                new InstantCommand(() -> {arm.setPIDlimits(-1, 1);}),
                new InstantCommand(() -> {arm.setPID(12, 0.0, 0.0);}), 
                new InstantCommand(() -> {intake.setSpeed(0.5);}),
                new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
                new WaitCommand(0.1),
                new InstantCommand(()-> {arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);}),
                new WaitCommand(0.0),
                new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
                new BargFligIntake(arm, intake, Constants.Presets.armBargeYeetRelease),
                // new WaitCommand(0.39),//WORKED at 0.2
                // new InstantCommand(() -> {intake.setSpeed(-1);}),
                new WaitCommand(0.75),
                new InstantCommand(() -> {elevator.setPIDlimits(-0.5, 0.5);}),
                new InstantCommand(() -> {arm.setPID(6, 0, 0);}),
                new InstantCommand(() -> {arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);}),
                new InstantCommand(() -> {arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);}),
                new InstantCommand(() -> {arm.setCurrentLimit(Constants.Arm.normalCurrentLimit);})
            )); 


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
    autoChooser.addOption("Outtake Test", new AutoIntakePower(intake, -1));
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
    controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.y().onTrue(Commands.runOnce(()->{
        AutoAlignDesitationDeterminer.seekingAlgea = !AutoAlignDesitationDeterminer.seekingAlgea;
    }));
    // controller.y().onFalse(Commands.runOnce(()->{
    //     AutoAlignDesitationDeterminer.seekingAlgea = false;
    // }));
    controller.back().onTrue(Commands.runOnce(
        () -> {
           GyroIOPigeon2.pigeon.setYaw(0.0);
        }));
    controller.start().onTrue(Commands.runOnce(
        () -> {
           drive.rezeroModulesRelativeEncoders();
        }));
    OI.ToggleDefenseMode().rising().ifHigh(()->{
        if(arm.defenseMode == false){
            arm.defenseMode = true;
            groundIntake.hoverPosition = Constants.Presets.groundIntakeStore;
            Constants.Presets.defenseDelay = 0.5;
        }
        else{
            arm.defenseMode = false;
            groundIntake.hoverPosition = Constants.Presets.groundIntakeHover;
            Constants.Presets.defenseDelay = 0.0;
        }
    });
    controller.
        leftBumper()
        .or(controller.rightBumper()).or(controller.x())
        .onTrue(
            Commands.runOnce(
                () -> {
                  System.out.println("ALIGNING-------------------------------------------");
                  // DriveCommands.autoAlign(drive).execute();
                  System.out.println(drive.getDefaultCommand());
                  // ------------

                  // -----------
                  drive.setDefaultCommand(
                      DriveCommands.autoAlign(drive, controller.leftBumper().getAsBoolean(), controller.x().getAsBoolean()));
                  System.out.println(drive.getDefaultCommand());

                  // ------------

        }));
    controller
        .leftBumper()
        .or(controller.rightBumper()).or(controller.x())
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
        .or(controller.rightBumper()).or(controller.x())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPoseDummy(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    //======== L3 ============
    OI.L3Algea().rising().ifHigh(()->{
        Command l3AlgeaCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftAlgeaL3);
            arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
        }));
        if(!arm.groundIntaking){
            l3AlgeaCommand.schedule();
        }
        else{
            arm.bufferedCommand = l3AlgeaCommand;
        }
    });
    OI.L3Algea().falling().ifHigh(()->{
        Command l3AlgeaCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
            arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
        }));
        if(!arm.groundIntaking){
            l3AlgeaCommand.schedule();
        }
        else{
            arm.bufferedCommand = l3AlgeaCommand;
        }
    });
    OI.L3Coral().rising().ifHigh(()->{
        Command l3CoralCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
        arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
        }));
        if(!arm.groundIntaking){
            l3CoralCommand.schedule();
        }
        else{
            arm.bufferedCommand = l3CoralCommand;
        }
    });
    //========== L2 ===============
    OI.L2Algea().rising().ifHigh(()->{
        Command l2AlgeaCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftAlgeaTeleopL2);
            arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
        }));
        if(!arm.groundIntaking){
            l2AlgeaCommand.schedule();
        }
        else{
            arm.bufferedCommand = l2AlgeaCommand;
        }
        
    });
    OI.L2Algea().falling().ifHigh(()->{
        Command l2AlgeaCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
        }));
        if(!arm.groundIntaking){
            l2AlgeaCommand.schedule();
        }
        else{
            arm.bufferedCommand = l2AlgeaCommand;
        }
    });
    OI.L2Coral().rising().ifHigh(()->{
        Command l2CoralCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
        }));
        if(!arm.groundIntaking){
            l2CoralCommand.schedule();
        }
        else{
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
    //     .x().or(()->leftButtonBoard.getRawButton(5)).or(() -> leftButtonBoard.getRawButtonReleased(4))
    //     .onFalse(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                     elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
    //                     arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
    //                 })));
    //========= L1 ==============
    // controller
    //     .a().or(()->leftButtonBoard.getRawButton(9))
    //     .onFalse(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //                     arm.setTargetAngle(Constants.Presets.armOuttakeL1, 0);
    //                 })));
    OI.l1Test().rising().ifHigh(()->{
        Command l1TestCommand =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.L1elevatorTest);
            arm.setTargetAngle(Constants.Presets.L1armtest, 0);
        }));
        if(!arm.groundIntaking){
            l1TestCommand.schedule();
        }
        else{
            arm.bufferedCommand = l1TestCommand;
        }
    });
    //========= Intake ==============
    OI.Intake().rising().ifHigh(()->{
        Command intakeCommand = Commands.sequence(
            new InstantCommand(() -> {
                
                elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
            }),
            new WaitCommand(0.25),
            new InstantCommand(() -> {
                groundIntake.setIntakePosition(Constants.Presets.groundIntakeStation);
                groundIntake.setIntakePower(0, 0);
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                arm.setTargetAngle(Constants.Presets.armIntakeAlt, 0);
            }),
            new WaitCommand(0.25),
            new InstantCommand(() -> {
                elevator.setElevatorPosition(Constants.Presets.liftIntakeAlt);
                
            }));
            if(!arm.groundIntaking){
                intakeCommand.schedule();
            }
            else{
                arm.bufferedCommand = intakeCommand;
            }         
        
        
    });
    // OI.Intake().rising().ifHigh(()->{
    //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
    //     arm.setTargetAngle(Constants.Presets.armIntakeLow, 0);
    // });
    OI.Processor().rising().ifHigh(()->{

        elevator.setElevatorPosition(Constants.Presets.liftIntake);
        arm.setTargetAngle(Constants.Presets.armCoral, 0);
    });
    OI.lolipop().rising().ifHigh(()->{
        Command lolipop =  Commands.sequence(new InstantCommand(()->{
            elevator.setElevatorPosition(Constants.Presets.liftIntake);
            arm.setTargetAngle(Constants.Presets.armLoliPop, 0);
        }));
        if(!arm.groundIntaking){
           lolipop.schedule();
        }
        else{
            arm.bufferedCommand = lolipop;
        }
    });

    OI.activateGroundIntake().rising().ifHigh(()->{
        if(arm.getTargetPosition() == Constants.Presets.armIntakeAlt + Constants.Presets.globalArmOffset){
            Constants.Presets.defenseDelay = 1;
        }
        else{
            Constants.Presets.defenseDelay = 0;
        }
        if(groundIntake.targetPosition == Constants.Presets.groundIntakeL1 ||groundIntake.targetPosition == Constants.Presets.groundIntakeStation || groundIntake.targetPosition == Constants.Presets.groundIntakeStore){
            Constants.Presets.defenseDelay = 1;
        }
        else{
            Constants.Presets.defenseDelay = 0;
        }
        
        Commands.sequence(
            new InstantCommand(() -> {
                arm.groundIntaking = true;
                if(arm.getTargetPosition() == Constants.Presets.armIntakeAlt + Constants.Presets.globalArmOffset){
                    elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
                    // Constants.Presets.defenseDelay = 2;
                }
                else{
                    elevator.setElevatorPosition(Constants.Presets.liftIntake);
                    // Constants.Presets.defenseDelay = 0;
                }
                
                arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                
               
            }),
            new WaitCommand(Constants.Presets.defenseDelay/2.0),
            new InstantCommand(() -> {
                
                groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
                groundIntake.setIntakePower(-0.85, 0.5);
                
            }),
            new WaitCommand(Constants.Presets.defenseDelay/3.5),
            new InstantCommand(() -> {
                elevator.setElevatorPosition(Constants.Presets.liftIntake);
                arm.setTargetAngle(Constants.Presets.armGroundTransfer, 0);
            })            
        ).schedule();
        
    });

    OI.activateGroundIntake().falling().ifHigh(()->{
        Commands.sequence(
            new InstantCommand(() -> {
                elevator.setElevatorPosition(Constants.Presets.liftIntake);
                arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
            }),
            new WaitCommand(Constants.Presets.defenseDelay/3),
            new InstantCommand(() -> {
                groundIntake.setIntakePosition(groundIntake.hoverPosition);
                groundIntake.setIntakePower(0, 0);
                arm.groundIntaking = false;
            })
        ).schedule();
        
    });

    OI.L1GroundIntake().rising().ifHigh(()->{
        Command l1gIntake = Commands.sequence(
            new InstantCommand(() -> {
                elevator.setElevatorPosition(Constants.Presets.liftIntake);
                arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
                groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
                groundIntake.setIntakePower(-0.85, -0.6);
            })
        );
        if(!arm.groundIntaking){
            l1gIntake.schedule();
         }
         else{
             arm.bufferedCommand = l1gIntake;
         }
    });
    OI.L1GroundIntake().falling().ifHigh(()->{
        Command l1gItnake = Commands.sequence(
            // new InstantCommand(() -> {
            //     elevator.setElevatorPosition(Constants.Presets.liftIntake);
            //     arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
            // }),
            // new WaitCommand(0.5),
            new InstantCommand(() -> {
                groundIntake.setIntakePosition(Constants.Presets.groundIntakeL1);
                groundIntake.setIntakePower(0, 0);
            })
        );
        if(!arm.groundIntaking){
            l1gItnake.schedule();
         }
         else{
             arm.bufferedCommand = l1gItnake;
         }
    });

    OI.groundIntakeOuttake().rising().ifHigh(()->{
        groundIntake.setIntakePower(0.2, -0.5);
    });
    OI.groundIntakeOuttake().falling().ifHigh(()->{
        groundIntake.setIntakePower(0, 0);
    });
    OI.groundIntakeIntake().rising().ifHigh(()->{
        groundIntake.setIntakePower(-0.2, 0.5);
    });
    OI.groundIntakeIntake().falling().ifHigh(()->{
        groundIntake.setIntakePower(0, 0);
    });
    //LOW INTAKE======================
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
    //BARGE===================
    OI.bargeStage().rising().ifHigh(()->{
        Commands.sequence(
            new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
            new WaitCommand(0.5),
            new InstantCommand(()-> {arm.setTargetAngle(Constants.Presets.armBargeStore, 0);})
        ).schedule();
    });
    OI.bargeYeet().rising().ifHigh(()->{
        Commands.sequence(
            new InstantCommand(() -> {arm.setCurrentLimit(85);}),
            // new InstantCommand(() -> {elevator.setPIDlimits(-1, 1);}),
            new InstantCommand(() -> {arm.setPIDlimits(-1, 1);}),
            new InstantCommand(() -> {arm.setPID(12, 0.0, 0.0);}), 
            new InstantCommand(() -> {intake.setSpeed(0.5);}),
            new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
            new WaitCommand(0.1),
            new InstantCommand(()-> {arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);}),
            new InstantCommand(() -> {arm.setPIDlimits(-1, 1);}),
            new WaitCommand(0.0),
            new InstantCommand(() -> {elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);}),
            new BargFligIntake(arm, intake, Constants.Presets.armBargeYeetRelease),
            // new WaitCommand(0.39),//WORKED at 0.2
            // new InstantCommand(() -> {intake.setSpeed(-1);}),
            new WaitCommand(0.75),
            new InstantCommand(() -> {elevator.setPIDlimits(-0.8, 0.8);}),
            new InstantCommand(() -> {arm.setPID(9, 0, 0);}),
            new InstantCommand(() -> {arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);}),
            new InstantCommand(() -> {arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);}),
            new InstantCommand(() -> {arm.setCurrentLimit(Constants.Arm.normalCurrentLimit);})
        ).schedule();
    });    
    //CLIMB===========================
    OI.startClimb().rising().ifHigh(()->{
        Commands.sequence(
            new InstantCommand(() -> {
                elevator.setElevatorPosition(Constants.Presets.liftClimb);
                arm.setTargetAngle(Constants.Presets.armClimb, 0);
                groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                climber.setTargetAngle(0, 0);
            })
        ).schedule();
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

    OI.ClimbStage1().rising().ifHigh(()->{
        climber.setCurrentLimit(20);
        climber.setTargetAngle(Constants.Presets.climberStage1, 0);
    });

    OI.ClimbStage2().rising().ifHigh(()->{
        Commands.sequence(
            new InstantCommand(()->{climber.setMotorBreak();}),
            new InstantCommand(()->{climber.setCurrentLimit(80);}),
            new InstantCommand(()->{climber.setTargetAngle(Constants.Presets.climberStage2, 0);}),
            new WaitCommand(1.5),
            new InstantCommand(()->{climber.setCurrentLimit(70);}),
            new WaitCommand(0.1),
            new InstantCommand(()->{climber.setCurrentLimit(60);}),
            new WaitCommand(0.1),
            new InstantCommand(()->{climber.setCurrentLimit(50);}),
            new WaitCommand(0.1),
            new InstantCommand(()->{climber.setCurrentLimit(40);}),
            new WaitCommand(0.1),
            new InstantCommand(()->{climber.setCurrentLimit(20);}),
            new WaitCommand(0.1),
            new InstantCommand(()->{climber.setCurrentLimit(0);})
        ).schedule(); 
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
