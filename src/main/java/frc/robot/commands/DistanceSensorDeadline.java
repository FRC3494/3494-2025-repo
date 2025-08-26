package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;

public class DistanceSensorDeadline extends Command {
  GroundIntake groundIntake;

  public DistanceSensorDeadline(GroundIntake groundIntake) {
    this.groundIntake = groundIntake;
    addRequirements(groundIntake);
  }

  @Override
  public boolean isFinished() {
    return groundIntake.getDistanceSensor() <= Constants.GroundIntake.CoralDistanceTheshold;
  }
}
