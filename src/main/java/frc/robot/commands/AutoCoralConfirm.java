package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Intake;

public class AutoCoralConfirm extends Command{
    Intake intake;

    public AutoCoralConfirm(Intake intake){
        this.intake = intake;
    }
    @Override 
    public boolean isFinished(){
        if(intake.getHasCoral()) return true;
        return false;
    }
    
}
