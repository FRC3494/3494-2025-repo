package frc.robot.subsystems.SuperStructure;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  SparkFlex armMotor;
  SparkFlexConfig armMotorConfig;
  double manualPower = 0;
  private double targetPosition;
  private RelativeEncoder encoder;

  public Command bufferedCommand = null;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(Constants.Arm.maxVelocity, Constants.Arm.maxAcceleration);

  private final ProfiledPIDController profiledPID =
      new ProfiledPIDController(6.0, 0.0, 0.0, armConstraints);

  private boolean useProfiledPID = false;

  public Arm() {
    armMotor = new SparkFlex(Constants.Arm.armMotor, MotorType.kBrushless);

    encoder = armMotor.getEncoder();

    armMotorConfig = new SparkFlexConfig();
    armMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.inverted(false);
    armMotorConfig.smartCurrentLimit(Constants.Arm.normalCurrentLimit);

    armMotorConfig.closedLoop.pid(9, 0, 0);
    armMotorConfig.closedLoop.outputRange(
        -Constants.Arm.normalPIDRange, Constants.Arm.normalPIDRange);
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotorConfig.closedLoopRampRate(0);
    armMotorConfig.openLoopRampRate(0);

    armMotorConfig.absoluteEncoder.inverted(false);
    armMotorConfig.absoluteEncoder.positionConversionFactor(1);
    armMotorConfig.absoluteEncoder.zeroOffset(Constants.Arm.ARM_OFFSET);
    armMotorConfig.absoluteEncoder.zeroCentered(false);

    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    profiledPID.setTolerance(Constants.Arm.positionTolerance);
    profiledPID.setGoal(getAbsoluteTicks());
  }

  public void setBrakes(IdleMode neutralMode) {
    this.armMotorConfig.idleMode(neutralMode);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetAngle(double ticks, double arbFFVoltage) {
    targetPosition = ticks + Constants.Presets.globalArmOffset;
    profiledPID.setGoal(targetPosition);
    useProfiledPID = true;
  }

  public void setPID(double p, double i, double d) {
    profiledPID.setPID(p, i, d);
  }

  public void useBuiltInPID(boolean useBuiltIn) {
    useProfiledPID = !useBuiltIn;
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Arm/Arm-Position", encoder.getPosition());
    // Logger.recordOutput("Arm/Arm-Encoder-Position", armMotor.getAbsoluteEncoder().getPosition());
    // Logger.recordOutput("Arm/Target-Position", targetPosition);
    // Logger.recordOutput("Arm/Manual-Power", manualPower);
    // Logger.recordOutput("Arm/Applied-Output", armMotor.getAppliedOutput());
    // Logger.recordOutput("Arm/Current-Limit", armMotor.configAccessor.getSmartCurrentLimit());
    // Logger.recordOutput("Arm/Applied-Current", armMotor.getOutputCurrent());
    // Logger.recordOutput("Arm/RPM", armMotor.getEncoder().getVelocity());
    // Logger.recordOutput("Arm/DefenseMode", defenseMode);
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
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void setPIDlimits(double lowerBound, double upperBound) {
    armMotorConfig.closedLoop.outputRange(lowerBound, upperBound);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCurrentLimit(int limit) {
    armMotorConfig.smartCurrentLimit(limit);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
