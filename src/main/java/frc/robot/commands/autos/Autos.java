package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.AutoPickupCoral;
import frc.robot.commands.IntakeGroundCoral;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;

public class Autos {
  private AutoFactory autoFactory;
  private Intake intake;
  private Elevator elevator;
  private Arm arm;
  private GroundIntake groundIntake;
  private Drive drive;

  private GroundIntakeCommands groundIntakeCommands = new GroundIntakeCommands();
  private CoralArmCommands coralArmCommands = new CoralArmCommands();
  private AlgaeArmCommands algaeArmCommands = new AlgaeArmCommands();
  private IntakeCommands intakeCommands = new IntakeCommands();

  public AutoRoutines AutoRoutines = new AutoRoutines();

  public Autos(
      AutoFactory autoFactory,
      Intake intake,
      Elevator elevator,
      Arm arm,
      GroundIntake groundIntake,
      Drive drive) {
    this.autoFactory = autoFactory;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.groundIntake = groundIntake;
    this.drive = drive;
  }

  public class AutoRoutines {
    public Command wraparound() {
      final String trajectoryName = "Wraparound";

      return Commands.sequence(
          autoFactory.resetOdometry(trajectoryName, 0),
          intakeCommands.coralIntake(),

          // Place first coral (preload)
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 0),
              Commands.sequence(groundIntakeCommands.freeArm(), coralArmCommands.L2Coral())),
          intakeCommands.coralOuttake(),
          new WaitCommand(0.5),

          // Pluck algae
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 1),
              Commands.sequence(
                  intakeCommands.stopIntake(),
                  algaeArmCommands.L3Algae(),
                  new WaitCommand(0.25),
                  intakeCommands.algaeIntake(),
                  groundIntakeCommands.groundIntakeDown())),
          new WaitCommand(0.5),

          // Pick up second coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 2),
              Commands.sequence(
                  new WaitCommand(0.55), intakeCommands.coralIntake(), new WaitCommand(0.5))),
          groundIntakeCommands.visionCoralPickup(),

          // Place second coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 3),
              Commands.sequence(new WaitCommand(1), coralArmCommands.L3Coral())),
          intakeCommands.coralOuttake(),
          new WaitCommand(0.2),

          // Pick up third coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 4),
              Commands.sequence(
                  new WaitCommand(0.2),
                  intakeCommands.stopIntake(),
                  new IntakeGroundCoral(groundIntake, arm, elevator, intake))),
          groundIntakeCommands.visionCoralPickup(),

          // Place third coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 5),
              Commands.sequence(new WaitCommand(0.5), coralArmCommands.L3Coral())),
          intakeCommands.coralOuttake());
    }
  }

  private class GroundIntakeCommands {
    private Command freeArm() {
      return Commands.sequence(
          new InstantCommand(
              () -> {
                groundIntake.setIntakePosition(Constants.Presets.groundIntakePop);
              }),
          new WaitCommand(0.5),
          new InstantCommand(
              () -> {
                arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
              }));
    }

    private Command groundIntakeDown() {
      return new InstantCommand(
          () -> {
            groundIntake.setIntakePosition(Constants.Presets.groundIntakeHover);
          });
    }

    private Command visionCoralPickup() {
      return new AutoPickupCoral(drive, groundIntake, arm, elevator, intake, 2);
    }
  }

  private class CoralArmCommands {
    private Command L2Coral() {
      return new InstantCommand(
          () -> {
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
          });
    }

    private Command L3Coral() {
      return new InstantCommand(
          () -> {
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL3);
            arm.setTargetAngle(Constants.Presets.armOuttakeL3, 0);
          });
    }
  }

  private class AlgaeArmCommands {
    private Command L3Algae() {
      return new InstantCommand(
          () -> {
            elevator.setElevatorPosition(Constants.Presets.liftAlgeaL3);
            arm.setTargetAngle(Constants.Presets.armAlgeaL3, 0);
          });
    }
  }

  private class IntakeCommands {
    private Command coralIntake() {
      return new AutoIntakePower(intake, -1);
    }

    private Command coralOuttake() {
      return new AutoIntakePower(intake, 0.75);
    }

    private Command algaeIntake() {
      return new AutoIntakePower(intake, 1);
    }

    private Command stopIntake() {
      return new AutoIntakePower(intake, 0);
    }
  }
}
