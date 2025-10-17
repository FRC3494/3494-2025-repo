package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.AutoPickupCoral;
import frc.robot.commands.BargFligIntake;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;

public class RobotCommands {
  private final Elevator elevator;
  private final Arm arm;
  private final GroundIntake groundIntake;
  private final Intake intake;
  private final Drive drive;

  public RobotCommands(
      Elevator elevator, Arm arm, GroundIntake groundIntake, Intake intake, Drive drive) {
    this.elevator = elevator;
    this.arm = arm;
    this.groundIntake = groundIntake;
    this.intake = intake;
    this.drive = drive;
  }

  public Command store() {
    return new InstantCommand(
        () -> {
          elevator.setElevatorPosition(Constants.Presets.liftIntake);
          arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
          groundIntake.setIntakePosition(Constants.Presets.groundIntakeStore);
        });
  }

  public Command armBrake() {
    return new InstantCommand(
        () -> {
          arm.setBrakes(IdleMode.kBrake);
        });
  }

  // ==================== Ground Intake ====================
  public Command freeArm() {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              groundIntake.setIntakePosition(Constants.Presets.groundIntakePop);
            }),
        new WaitUntilCommand(
            () -> groundIntake.getPivotPosition() < Constants.GroundIntake.safePosition),
        new InstantCommand(
            () -> {
              arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
            }));
  }

  public Command groundIntakeHover() {
    return new InstantCommand(
        () -> {
          groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
        });
  }

  public Command groundIntakeStore() {
    return new InstantCommand(
        () -> {
          groundIntake.setIntakePosition(Constants.Presets.groundIntakeStore);
        });
  }

  public Command stopGroundIntake() {
    return new InstantCommand(
        () -> {
          groundIntake.setIntakePower(0, 0);
        });
  }

  // =================== Coral ====================
  public Command coralIntake() {
    return new AutoIntakePower(intake, -1);
  }

  public Command coralOuttake() {
    return new AutoIntakePower(intake, 0.75);
  }

  public Command visionCoralPickup() {
    return new AutoPickupCoral(drive, groundIntake, arm, elevator, intake, 2);
  }

  public Command L2Coral() {
    return new InstantCommand(
        () -> {
          elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
          arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
        });
  }

  public Command L3Coral() {
    return new InstantCommand(
        () -> {
          elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
          arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
        });
  }

  // =================== Algae ====================
  public Command algaeIntake() {
    return new AutoIntakePower(intake, 1);
  }

  public Command algaeOuttake() {
    return new AutoIntakePower(intake, -1);
  }

  public Command stopIntake() {
    return new AutoIntakePower(intake, 0);
  }

  public Command L2Algae() {
    return new InstantCommand(
        () -> {
          elevator.setElevatorPosition(Constants.Presets.liftIntake);
          arm.setTargetAngle(Constants.Presets.armAlgeaL2, 0);
        });
  }

  public Command L3Algae() {
    return new InstantCommand(
        () -> {
          elevator.setElevatorPosition(Constants.Presets.liftAlgeaL3);
          arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
        });
  }

  public Command preBarge() {
    return Commands.sequence(
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
            }));
  }

  public Command barge() {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              arm.setCurrentLimit(75);
              intake.setSpeed(0.5);
              arm.setPIDlimits(-0.7, 0.7);
              arm.setPID(8, 0.0, 0.0);
              intake.setSpeed(0.0); // 0.75
              elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
              arm.setTargetAngle(Constants.Presets.armBargeYeet, 0);
            }),
        new WaitCommand(0.0),
        new BargFligIntake(arm, intake, Constants.Presets.armBargeYeetRelease),
        // new WaitCommand(0),//.39),//WORKED at 0.2
        // new InstantCommand(() -> {intake.setSpeed(-1);}),
        new WaitCommand(0.75),
        new InstantCommand(
            () -> {
              elevator.setPIDlimits(-0.8, 0.8);
              arm.setPID(9, 0, 0);
              arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
              arm.setCurrentLimit(Constants.Arm.normalCurrentLimit);
            }));
  }
}
