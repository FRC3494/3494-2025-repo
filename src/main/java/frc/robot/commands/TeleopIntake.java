package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Intake;

public class TeleopIntake extends Command {
  private Intake intake;

  public TeleopIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    // TODO: might have to invert intake speeds/directions
    if (OI.intakeIn() > 0) {
      intake.setSpeed(Math.pow(OI.intakeIn(), 2));
    } else if (OI.intakeOut() > 0) {
      intake.setSpeed(-1 * Math.pow(OI.intakeOut(), 2));
    } else {
      intake.setSpeed(0);
    }
  }

  public boolean isFinished() {
    return false;
  }

  public void end() {
    intake.setSpeed(0);
  }

  public void interrupted() {
    end();
  }
}
