package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Intake;

public class TeleopIntake extends Command {
  private Intake intake;
  double lastPower;

  public TeleopIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    // TODO: might have to invert intake speeds/directions
    double intakePower = Math.copySign(Math.pow(OI.getIntakePower(), 2), OI.getIntakePower());
    if(intakePower == 0){
      intakePower = 0.075* Math.copySign(1 ,lastPower) ;
    }
    else{
      lastPower = intakePower;
    }
    intake.setSpeed(intakePower);
    Logger.recordOutput("Intake/Intake-Power-Command", -1 * Math.pow(OI.getIntakePower(), 2));
      
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
