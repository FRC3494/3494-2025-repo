package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Intake;

public class AutoIntakePower extends Command {
    Intake intake;
    double power;
    Timer timer = new Timer();
    public AutoIntakePower(Intake intake, double power){
        this.intake = intake;
        this.power = power;
        timer.start();
    }
    @Override
    public void execute(){
        intake.setSpeed(power);
    }
    @Override
    public boolean isFinished(){
        if(timer.hasElapsed(0.5)){
            return true;
        }
        return false;
    }
}