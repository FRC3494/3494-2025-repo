package frc.robot.subsystems.SuperStructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  SparkFlex armMotor;
  SparkFlexConfig armMotorConfig;
  double manualPower = 0;
  private double targetPosition;

  public boolean groundIntaking = false;
  public Command bufferedCommand = null;

  public boolean defenseMode =
      switch (Constants.DRIVE_MODE) {
        case DEMO, DEMO_AUTOALIGN -> false;
        default -> true;
      };

  public Arm() {
    armMotor = new SparkFlex(Constants.Arm.armMotor, MotorType.kBrushless);
    armMotorConfig = new SparkFlexConfig();
    armMotorConfig.idleMode(IdleMode.kCoast);
    armMotorConfig.inverted(false);
    armMotorConfig.smartCurrentLimit(Constants.Arm.normalCurrentLimit);
    armMotorConfig.closedLoop.pid(9, 0, 0);
    armMotorConfig.closedLoop.outputRange(
        -Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange); // -.45, .45);
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotorConfig.closedLoopRampRate(0);
    armMotorConfig.openLoopRampRate(0);
    armMotorConfig.absoluteEncoder.zeroOffset(Constants.Arm.absoluteEncoderOffset);
    armMotorConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0, 1);
    // armMotorConfig
    //     .softLimit
    //     .forwardSoftLimit(Constants.Arm.forwardSoftLimit)
    //     .forwardSoftLimitEnabled(true)
    //     .reverseSoftLimit(Constants.Arm.reverseSoftLimit)
    //     .reverseSoftLimitEnabled(true);

    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setBrakes(IdleMode neutralMode) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(neutralMode);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    // targetPosition = ticks + Constants.Presets.globalArmOffset;

    if (targetPosition < 0) {}

    // armMotor
    //     .getClosedLoopController()
    //     .setReference(
    //         ticks + Constants.Presets.globalArmOffset,
    //         SparkMax.ControlType.kPosition,
    //         ClosedLoopSlot.kSlot0);
  }

  public void setPID(double p, double i, double d) {
    armMotorConfig.closedLoop.pid(p, i, d);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/ArmRelPosition", armMotor.getEncoder().getPosition());
    Logger.recordOutput("Arm/ArmAbsPosition", armMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Arm/Target-Position", targetPosition);
    Logger.recordOutput("Arm/Manual-Power", manualPower);
    Logger.recordOutput("Arm/Applied-Output", armMotor.getAppliedOutput());
    // Logger.recordOutput("Arm/Current-Limit", armMotor.configAccessor.getSmartCurrentLimit());
    Logger.recordOutput("Arm/Applied-Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm/RPM", armMotor.getEncoder().getVelocity());
    Logger.recordOutput("Arm/DefenseMode", defenseMode);
    Logger.recordOutput("Arm/MotorTemp", armMotor.getMotorTemperature());
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
    // return armMotor.getEncoder().getPosition();
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getPosition() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  public void setPIDlimits(double lowerBound, double upperBound) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.outputRange(lowerBound, upperBound);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void updatePIDLimits() {
    switch (Constants.DRIVE_MODE) {
      case DEMO, DEMO_AUTOALIGN -> {
        setPIDlimits(-0.4, 0.4);
      }
      default -> {
        setPIDlimits(-Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
      }
    }
  }

  public void setCurrentLimit(int limit) {
    armMotorConfig.smartCurrentLimit(limit);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
