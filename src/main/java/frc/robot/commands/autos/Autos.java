package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.RobotCommands;
import frc.robot.commands.IntakeGroundCoral;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;

public class Autos {
  private final AutoFactory autoFactory;
  private final Intake intake;
  private final Elevator elevator;
  private final Arm arm;
  private final GroundIntake groundIntake;
  private final Drive drive;

  private final RobotCommands robotCommands;

  public final AutoRoutines AutoRoutines = new AutoRoutines();

  public Autos(
      AutoFactory autoFactory,
      Intake intake,
      Elevator elevator,
      Arm arm,
      GroundIntake groundIntake,
      Drive drive,
      RobotCommands robotCommands) {
    this.autoFactory = autoFactory;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.groundIntake = groundIntake;
    this.drive = drive;
    this.robotCommands = robotCommands;
  }

  public class AutoRoutines {
    public Command wraparound() {
      final String trajectoryName = "Wraparound";

      return Commands.sequence(
          autoFactory.resetOdometry(trajectoryName, 0),
          robotCommands.coralIntake(),

          // Place 1st coral (preload)
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 0),
              Commands.sequence(robotCommands.freeArm(), robotCommands.L2Coral())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.5),

          // Pluck algae
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 1),
              Commands.sequence(
                  robotCommands.stopIntake(),
                  robotCommands.L3Algae(),
                  new WaitCommand(0.25),
                  robotCommands.algaeIntake(),
                  robotCommands.groundIntakeDown())),
          new WaitCommand(0.5),

          // Pickup 2nd coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 2),
              Commands.sequence(
                  new WaitCommand(0.55), robotCommands.coralIntake(), new WaitCommand(0.5))),
          robotCommands.visionCoralPickup(),

          // Place 2nd coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 3),
              Commands.sequence(new WaitCommand(1), robotCommands.L3Coral())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.2),

          // Pick up 3rd coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 4),
              Commands.sequence(
                  new WaitCommand(0.2),
                  robotCommands.stopIntake(),
                  new IntakeGroundCoral(groundIntake, arm, elevator, intake))),
          robotCommands.visionCoralPickup(),

          // Place 3rd coral
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 5),
              Commands.sequence(new WaitCommand(0.5), robotCommands.L3Coral())),
          robotCommands.coralOuttake());
    }

    public Command barge() {
      final String trajectoryName = "Barge";

      return Commands.sequence(
          autoFactory.resetOdometry(trajectoryName, 0),
          robotCommands.coralIntake(),

          // Place 1st coral (preload)
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 0),
              Commands.sequence(
                  robotCommands.freeArm(), new WaitCommand(0.5), robotCommands.L3Coral())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.25),
          robotCommands.stopIntake(),

          // Pluck 1st algae
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 1),
              Commands.sequence(
                  new WaitCommand(0.3), robotCommands.L2Algae(), robotCommands.algaeIntake())),
          new WaitCommand(0.5),

          // Barge
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 2),
              Commands.sequence(new WaitCommand(0.5), robotCommands.preBarge())),
          new WaitCommand(0.505),
          robotCommands.barge(),
          robotCommands.L3Algae(),

          // Pluck 2nd Algae
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 3),
              Commands.sequence(new WaitCommand(0.3), robotCommands.algaeIntake())),
          new WaitCommand(0.5),

          // End
          Commands.parallel(autoFactory.trajectoryCmd(trajectoryName, 4), robotCommands.store()));
    }
  }
}
