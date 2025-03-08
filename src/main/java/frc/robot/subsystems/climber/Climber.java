package frc.robot.subsystems.climber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberIO.ClimberMode;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  SparkMax climberMotor;

  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  public ClimberIO climberIO;
  public double prevTicks;

  public Climber() {
    climberMotor = new SparkMax(Constants.Climber.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig.idleMode(IdleMode.kCoast);
    climberMotorConfig.inverted(false);
    climberMotorConfig.closedLoop.pid(2, 0, 0);
    climberMotorConfig.closedLoop.outputRange(-1, 1);
    climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotorConfig.smartCurrentLimit(100);

    climberMotor.configure(
        climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberIO = new ClimberIO(climberMotor);
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (inputs.mode == ClimberMode.Automatic) {
      // Only update if it's a new value to not fill up can bus
      if (inputs.targetPosition != prevTicks) {
        climberMotor
            .getClosedLoopController()
            .setReference(
                inputs.targetPosition, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
    }

    // if (inputs.mode == ClimberMode.Manual) {
    //   if (inputs.climberPosition > -1) {
    //     climberMotor.set(inputs.power);
    //   }
    // }

    prevTicks = inputs.targetPosition;
  }

  public void setMotorPower(double power) {
    inputs.mode = ClimberMode.Manual;
    inputs.power = Math.max(Math.min(power, 1), -1);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    inputs.mode = ClimberMode.Automatic;
    inputs.targetPosition = ticks;
  }
}
