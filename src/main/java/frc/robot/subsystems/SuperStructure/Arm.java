package frc.robot.subsystems.SuperStructure;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  SparkMax armMotor;
  SparkMaxConfig armMotorConfig;
  
  double manualPower = 0;
  private double targetPosition;
  private RelativeEncoder encoder;

  public Arm() {
    armMotor = new SparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kCoast);
    armMotorConfig.inverted(false);
    armMotorConfig.closedLoop.pid(2, 0, 0);
    armMotorConfig.closedLoop.outputRange(-0.6, 0.6);
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    encoder = armMotor.getEncoder();
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setBrakes(IdleMode neutralMode) {
    this.armMotorConfig.idleMode(neutralMode);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    targetPosition = ticks;
    armMotor.getClosedLoopController().setReference(ticks, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    
  }

  @Override
  public void periodic() {
    // if (DriverStation.isEnabled()) this.setBrakes(IdleMode.kBrake);

    Logger.recordOutput("Arm/Arm-Position", encoder.getPosition());
    Logger.recordOutput("Arm/Arm-Encoder-Position", armMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Arm/Target-Position", targetPosition);
    Logger.recordOutput("Arm/Manual-Power", manualPower);
    Logger.recordOutput("Arm/Applied-Power", armMotor.getAppliedOutput());
    

  }

  // public void setMotorPower(double power) {
  //   power = Math.max(Math.min(power, 1), -1);
  //   manualPower = power;
  //   // System.out.println(manualPower);
  //   armMotor.set(manualPower);
  // }

  public double getManualMotorPower() {
    return manualPower;
  }

  public double getRelativeTicks() {
    return armMotor.getEncoder().getPosition();
  }

  public double getAbsoluteTicks() {
    return armMotor.getEncoder().getPosition();
    // return armMotor.getAbsoluteEncoder().getPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }
}
