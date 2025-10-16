package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;

public class IntakeGroundCoral extends Command {

  Arm arm;
  Elevator elevator;
  Intake intake;
  GroundIntake groundIntake;

  public IntakeGroundCoral(GroundIntake groundIntake, Arm arm, Elevator elevator, Intake intake) {

    this.groundIntake = groundIntake;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;

    addRequirements(groundIntake);
    addRequirements(arm);
    addRequirements(elevator);
    addRequirements(intake);
  }

  @Override
  public void initialize() {

    elevator.setElevatorPosition(Constants.Presets.liftIntake);
    groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
    groundIntake.setIntakePower(-0.85, 0.85);
    intake.setSpeed(-0.75);
    arm.setTargetAngle(
        groundIntake.wanttoPOP
            ? Constants.Presets.armGroundTransferWithPop
            : Constants.Presets.armGroundTransfer,
        0);
  }

  @Override
  public void end(boolean interupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
