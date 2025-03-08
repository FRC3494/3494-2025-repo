package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SuperStructure.Intake;
public class AutoIntakeDeadline extends Command {
    Intake intake;


    public AutoIntakeDeadline(Intake intake) {
        this.intake = intake;        
    }

    @Override
    public boolean isFinished(){
        if(intake.getHasCoral()) return true;
        return false;
        
    } 
}