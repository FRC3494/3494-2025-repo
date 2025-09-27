package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.LEDPattern;
import frc.robot.OI;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SuperStructure.Arm;
import frc.robot.subsystems.SuperStructure.Intake;

public class TeleopIntake extends Command {
  private Intake intake;
  private Arm arm;
  private LEDs leds;
  double lastPower;
  private double armPower = 0;
  private double lastIntakePower = 1;
  private boolean holding_algea = false;
  private Timer algeaTimer = new Timer();

  private boolean donePIdchange = false;

  public TeleopIntake(Intake intake, Arm arm, LEDs leds) {
    this.intake = intake;
    this.arm = arm;
    this.leds = leds;
    addRequirements(intake);
    addRequirements(arm);
    addRequirements(leds);
  }

  @Override
  public void execute() {
    // TODO: might have to invert intake speeds/directions
    double intakePower = Math.copySign(Math.pow(OI.getIntakePower(), 2), OI.getIntakePower());
    if (intakePower == 0) {
      intakePower = 0.1 * Math.copySign(1, lastPower);
    } else {
      lastPower = intakePower;
    }
    if (arm.getTargetPosition()
        == Constants.Presets.armOuttakeL1 + Constants.Presets.globalArmOffset) {
      intakePower *= 0.3;
      AutoAlignDesitationDeterminer.placingAtL1 = true;
    }
    if (arm.getTargetPosition()
        == Constants.Presets.L1armtest + Constants.Presets.globalArmOffset) {
      intakePower *= 0.75;
    } else {
      // AutoAlignDesitationDeterminer.placingAtL1 = false;
    }
    if (arm.getTargetPosition()
            == Constants.Presets.armProcessor + Constants.Presets.globalArmOffset
        || arm.getTargetPosition()
            == Constants.Presets.armBargeStore + Constants.Presets.globalArmOffset) {
      // arm.setPIDlimits(-0.8, 0.8);
      holding_algea = true;
      // algeaTimer.start();
      // System.out.println("limiting!!!!!!!!!");
      // if (arm.getAbsoluteTicks() < 0.7) {

      //   arm.setPIDlimits(-0.4, 0.4);
      // }
      if (arm.getAbsoluteTicks() > 0.7
          && donePIdchange == false
          && arm.getTargetPosition()
              == Constants.Presets.armBargeStore + Constants.Presets.globalArmOffset) {
        arm.setPIDlimits(-0.4, 0.4);
        donePIdchange = true;
      }
    } else {
      // arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
      algeaTimer.stop();
      holding_algea = false;
      donePIdchange = false;
    }

    // Positive power is coral outtake
    if ((arm.getTargetPosition()
            == Constants.Presets.armOuttakeL1 + Constants.Presets.globalArmOffset)
        || (arm.getTargetPosition()
            == Constants.Presets.armOuttakeL2 + Constants.Presets.globalArmOffset)
        || (arm.getTargetPosition()
            == Constants.Presets.armOuttakeL3 + Constants.Presets.globalArmOffset)) {
      if (intakePower > 0) {
        leds.setPattern(LEDPattern.DEPOSITED).schedule();
      } else if (lastIntakePower > 0 && OI.getIntakePower() == 0) {
        leds.setPattern(LEDPattern.NONE).schedule();
      }
    }

    if (OI.L1GroundIntake().getAsBoolean()) {
      intakePower = 0;
    }

    // else{
    //   arm.setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
    // }
    if (intakePower != lastIntakePower || OI.primaryController.getAButton()) {
      if (arm.getTargetPosition()
          == Constants.Presets.armGroundTransfer + Constants.Presets.globalArmOffset) {
        intake.setSpeed(intakePower);
      } else {
        intake.setSpeed(intakePower * Constants.Intake.speedScalar);
      }
    } else if (holding_algea && OI.deadband(intakePower, 0.5) == 0) {
      boolean isIntaking = ((int) (algeaTimer.get() * 10)) % 2 == 1;
      intake.setSpeed(((isIntaking) ? 1 : 0));
    }
    lastIntakePower = intakePower;

    Logger.recordOutput("Intake/Intake-Power-Command", -1 * Math.pow(OI.getIntakePower(), 2));

    // armPower = OI.deadband(OI.getArmPower(), 0.05);
    // Logger.recordOutput("Arm/Manual-Power-Command", armPower);
    // if (armPower != 0 || (arm.getManualMotorPower() != 0 && armPower == 0)) {
    //   // arm.setMotorPower(armPower*0.2);
    //   Logger.recordOutput("Arm/Manual-index-Command", armPower * Constants.Arm.manualPowerPOS);
    //   arm.setTargetAngle(
    //       arm.getTargetPosition()
    //           + armPower * Constants.Arm.manualPowerPOS
    //           - Constants.Presets.globalArmOffset,
    //       0);
    // }
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
