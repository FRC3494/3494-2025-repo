package frc.robot.commands.deadlines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.enums.ComparisonDirection;
import frc.robot.subsystems.SuperStructure.Arm;

public class ArmPositionDeadline extends Command {
  Arm arm;
  double position;
  ComparisonDirection direction;
  double DEADBAND = 0.05;

  public ArmPositionDeadline(Arm arm, double position, ComparisonDirection direction) {
    this.arm = arm;
    this.position = position;
    this.direction = direction;
    addRequirements(arm);
  }

  @Override
  public boolean isFinished() {
    if (direction == ComparisonDirection.GREATER_THAN) {
      return arm.getRelativeTicks() >= position - DEADBAND;
    } else {
      return arm.getRelativeTicks() <= position + DEADBAND;
    }
  }
}
