package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoIntakePower;
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

  private SuperstructureCommands SuperstructureCommands = new SuperstructureCommands();
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
          new AutoIntakePower(intake, -1),

          // Go to first piece
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 0),
              Commands.sequence(
                  SuperstructureCommands.freeArm(),
                  new InstantCommand(
                      () -> {
                        arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
                      }))),
          new AutoIntakePower(intake, 0.75),
          new WaitCommand(0.5),
          autoFactory.trajectoryCmd(trajectoryName, 1),
          new WaitCommand(0.5),
          autoFactory.trajectoryCmd(trajectoryName, 2),
          // new AutoPickupCoral(drive, groundIntake, arm, elevator, intake, 2),
          // new InstantCommand(
          //     () -> {
          //       arm.setTargetAngle(Constants.Presets.armSafePosition, 0);
          //     }),
          new WaitCommand(0.5),
          autoFactory.trajectoryCmd(trajectoryName, 3),
          new WaitCommand(0.5),
          autoFactory.trajectoryCmd(trajectoryName, 4));
    }
  }

  private class SuperstructureCommands {
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

    private Command L2Outtake() {
      return new InstantCommand(
          () -> {
            elevator.setElevatorPosition(Constants.Presets.liftOuttakeL2);
            arm.setTargetAngle(Constants.Presets.armOuttakeL2, 0);
          });
    }
  }
}
