package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Arm;

public class TeleopArm extends Command {
  Arm arm;
  private double armPower = 0;

  public TeleopArm(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    // armPower = OI.deadband(OI.getArmPower(), 0.05);
    // System.out.println("test");
    // Logger.recordOutput("Arm/Manual-Power-Command", armPower);
    // // if(armPower != 0 || (arm.getManualMotorPower() != 0 && armPower == 0)){
    // //     // arm.setMotorPower(armPower*0.2);
    // //     arm.setTargetAngle(arm.getTargetPosition()+armPower*Constants.Arm.manualPowerPOS,0);
    // // }

  }
}
