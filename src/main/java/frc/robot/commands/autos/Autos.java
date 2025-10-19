package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotCommands;
import frc.robot.commands.IntakeGroundCoral;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SeanMathUtil;

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
      int pathIndex = 0;

      return Commands.sequence(
          new InstantCommand(
              () -> {
                System.out.println(Timer.getMatchTime());
              }),
          autoFactory.resetOdometry(trajectoryName, 0),
          robotCommands.coralIntake(),

          // Place 1st coral (preload): B-L2
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  robotCommands.freeArm(),
                  robotCommands.L2Coral(),
                  new WaitUntilCommand(
                      () ->
                          SeanMathUtil.compareArmPosition(
                              arm.getPosition(), Constants.Arm.safePosition, false)),
                  robotCommands.groundIntakeStore())),
          new WaitCommand(0.25),
          robotCommands.coralOuttake(),
          new WaitCommand(0.5),

          // Pluck algae: AB
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  robotCommands.stopIntake(),
                  robotCommands.L3Algae(),
                  new WaitCommand(0.25),
                  robotCommands.algaeIntake(),
                  robotCommands.groundIntakeHover())),
          new WaitCommand(0.5),

          // Release algae
          autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
          robotCommands.algaeOuttake(),

          // Pickup 2nd coral: Right preset coral from DS
          autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
          robotCommands.visionCoralPickup(),
          new WaitCommand(0.5),

          // Place 2nd coral: B-L3
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  new WaitCommand(0.5),
                  robotCommands.groundIntakeHover(),
                  robotCommands.L3Coral(),
                  robotCommands.stopGroundIntake())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.2),

          // Pick up 3rd coral: Center preset coral from DS
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  new WaitCommand(0.2),
                  robotCommands.stopIntake(),
                  new IntakeGroundCoral(groundIntake, arm, elevator, intake))),
          robotCommands.visionCoralPickup(),
          new WaitCommand(0.25),

          // Place 3rd coral: A-L3
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  new WaitCommand(0.75),
                  robotCommands.groundIntakeHover(),
                  robotCommands.L3Coral(),
                  robotCommands.stopGroundIntake())),
          robotCommands.coralOuttake(),
          new WaitCommand(1),

          // Back up
          autoFactory.trajectoryCmd(trajectoryName, pathIndex++),

          // End
          new InstantCommand(
              () -> {
                System.out.println(Timer.getMatchTime());
              }));
    }

    public Command barge() {
      final String trajectoryName = "Barge";

      return Commands.sequence(
          autoFactory.resetOdometry(trajectoryName, 0),
          robotCommands.coralIntake(),

          // Place 1st coral (preload): H-L3
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 0),
              Commands.sequence(
                  robotCommands.freeArm(), new WaitCommand(0.5), robotCommands.L3Coral())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.25),
          robotCommands.stopIntake(),

          // Pluck 1st algae: GH
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

          // Pluck 2nd Algae: IJ
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, 3),
              Commands.sequence(new WaitCommand(0.3), robotCommands.algaeIntake())),
          new WaitCommand(0.5),

          // End
          Commands.parallel(autoFactory.trajectoryCmd(trajectoryName, 4), robotCommands.store()));
    }

    public Command backside() {
      final String trajectoryName = "Backside";
      int pathIndex = 0;

      return Commands.sequence(
          autoFactory.resetOdometry(trajectoryName, 0),
          robotCommands.coralIntake(),

          // Place 1st coral (preload): H-L3
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  robotCommands.freeArm(),
                  new WaitCommand(0.5),
                  robotCommands.L3Coral(),
                  new WaitUntilCommand(
                      () ->
                          SeanMathUtil.compareArmPosition(
                              arm.getPosition(), Constants.Arm.safePosition, false)),
                  robotCommands.groundIntakeStore())),
          robotCommands.coralOuttake(),
          new WaitCommand(0.25),
          robotCommands.stopIntake(),

          // Pluck 1st algae: GH
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(
                  new WaitCommand(0.3), robotCommands.L2Algae(), robotCommands.algaeIntake())),
          new WaitCommand(0.5),

          // Hold algae
          Commands.parallel(
              autoFactory.trajectoryCmd(trajectoryName, pathIndex++),
              Commands.sequence(new WaitCommand(0.6), robotCommands.store())),
          robotCommands.armBrake());
    }
  }
}
