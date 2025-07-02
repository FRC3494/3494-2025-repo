package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Elevator;
import frc.robot.subsystems.SuperStructure.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class IntakeGroundCoral extends Command {

    Arm arm;
    Elevator elevator;
    Intake intake; 
    GroundIntake groundIntake;


    public IntakeGroundCoral( GroundIntake groundIntake, Arm arm, Elevator elevator, Intake intake) {

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
    public void initialize(){

        elevator.setElevatorPosition(Constants.Presets.liftIntake);
        groundIntake.setIntakePosition(Constants.Presets.groundIntakeIntake);
        groundIntake.setIntakePower(-0.85, 0.85);
        intake.setSpeed(-0.75);
        arm.setTargetAngle(Constants.Presets.armGroundTransfer, 0);


    }
   
    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
