package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Intake;

public class BargFligIntake extends Command {

  Arm arm;
  Intake intake;
  double releaseThreshold;
  private boolean readyToOuttake = false;

  public BargFligIntake(Arm arm, Intake intake, double releaseThreshold) {
    this.releaseThreshold = releaseThreshold;
    this.arm = arm;
    this.intake = intake;
  }

  @Override
  public void execute() {
    if (arm.getAbsoluteTicks() <= releaseThreshold) {
      System.out.println(
          "Outtaking!!!!!!!!!!!!!!!!!! ----------------------------------------------------------------------------------------------------");
      readyToOuttake = true;
      intake.setSpeed(-1);
    }
  }

  @Override
  public boolean isFinished() {
    return readyToOuttake;
  }
}
