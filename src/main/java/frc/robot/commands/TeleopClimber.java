package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.SuperStructure.Arm;

public class TeleopClimber extends Command {
    Climber climber;
    private double climberPower = 0;
    public TeleopClimber(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }
    @Override
    public void execute() {
        climberPower = OI.deadband(OI.getClimberPower(), 0.05);
        Logger.recordOutput("Climber/Manual-Power-Command", climberPower);
        // if(climberPower != 0 || (climber.getManualMotorPower() != 0 && climberPower == 0)){
        //     climber.setTargetAngle(climber.getTargetPosition()+climberPower*Constants.Climber.manualPowerPOS, climberPower);
        // }

        //We have a ratchet now!
        // climber.setMotorPower(climberPower);
        

    }
    
}
