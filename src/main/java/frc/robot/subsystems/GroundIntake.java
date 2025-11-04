package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {
  private SparkFlex pivotMotor;
  private PIDController pivotController;

  private SparkMax frontIntakeMotor;
  private SparkMaxConfig frontIntakeMotorConfig;
  private SparkMax backIntakeMotor;
  private SparkMaxConfig backIntakeMotorConfig;
  public boolean wanttoPOP = true;
  public Double targetPosition = null;
  public double hoverPosition =
      switch (Constants.DRIVE_MODE) {
        case DEMO, DEMO_AUTOALIGN -> Constants.Presets.groundIntakeHover;
        default -> Constants.Presets.groundIntakeStore;
      };

  public boolean fastPID = false;

  // Create instance of Time-Of_Flight driver for device 1
  private final TimeOfFlight m_rangeSensor =
      new TimeOfFlight(Constants.GroundIntake.distanceSensorDeviceNumber);

  public GroundIntake() {
    pivotMotor = new SparkFlex(Constants.GroundIntake.pivotMotor, MotorType.kBrushless);
    SparkFlexConfig pivotMotorConfig = new SparkFlexConfig();
    frontIntakeMotor = new SparkMax(Constants.GroundIntake.frontIntakeMotor, MotorType.kBrushless);
    backIntakeMotor = new SparkMax(Constants.GroundIntake.backIntakeMotor, MotorType.kBrushless);

    targetPosition = pivotMotor.getAbsoluteEncoder().getPosition();
    pivotMotorConfig = new SparkFlexConfig();
    frontIntakeMotorConfig = new SparkMaxConfig();
    backIntakeMotorConfig = new SparkMaxConfig();

    pivotMotorConfig.smartCurrentLimit(45);
    pivotMotorConfig.inverted(false);
    pivotController = new PIDController(4.6, 0, 0);

    pivotController.setSetpoint(getPivotPosition());
    targetPosition = getPivotPosition();
    // pivotController.enableContinuousInput(0, 1);

    // pivotMotorConfig.closedLoop.pidf(8.0, 0, 0, 0.4);
    // pivotMotorConfig.closedLoop.outputRange(-0.9, 0.9); // 0.8// 0.7
    // pivotMotorConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0, 1);
    // pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // pivotMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(6.0);

    pivotMotorConfig.idleMode(IdleMode.kBrake);

    frontIntakeMotorConfig.idleMode(IdleMode.kBrake);
    frontIntakeMotorConfig.smartCurrentLimit(45);
    frontIntakeMotorConfig.inverted(false);

    backIntakeMotorConfig.idleMode(IdleMode.kBrake);
    backIntakeMotorConfig.smartCurrentLimit(45);
    backIntakeMotorConfig.inverted(false);

    pivotMotor.configure(
        pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontIntakeMotor.configure(
        frontIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backIntakeMotor.configure(
        backIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure time of flight sensor for short ranging mode and sample
    // distance every 40 ms
    m_rangeSensor.setRangingMode(RangingMode.Short, 40);
  }

  public void setIntakePosition(double position) {
    // pivotMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    pivotController.setSetpoint(position);
    targetPosition = position;
  }

  public double getPivotPosition() {
    if (pivotMotor.getAbsoluteEncoder().getPosition() > 0.9) {
      return pivotMotor.getAbsoluteEncoder().getPosition() - 1;
    } else {
      return pivotMotor.getAbsoluteEncoder().getPosition();
    }
  }

  public void setIntakePower(double front, double back) {
    frontIntakeMotor.set(front);
    backIntakeMotor.set(back);
  }

  @Override
  public void periodic() {

    // double motorpower = pivotMotorPIDLoop.calculate(targetPosition,
    // pivotMotor.getAbsoluteEncoder().getPosition());
    // // motorpower = 0.4;
    // motorpower = Math.max(motorpower, -0.8);
    // motorpower = Math.min(motorpower, 0.8);
    // motorpower = -motorpower;
    // if(Math.abs(targetPosition-pivotMotor.getAbsoluteEncoder().getPosition())<0.015){
    //   motorpower = 0;
    // }
    // pivotMotor.set(motorpower);
    if (targetPosition != null) {
      double pidPower = pivotController.calculate(getPivotPosition(), targetPosition);
      if (!fastPID && targetPosition < getPivotPosition()) {
        pidPower =
            MathUtil.clamp(
                pidPower,
                -Constants.GroundIntake.downPIDRange,
                Constants.GroundIntake.downPIDRange);
      } else if (!fastPID) {
        pidPower =
            MathUtil.clamp(
                pidPower, -Constants.GroundIntake.upPIDRange, Constants.GroundIntake.upPIDRange);
      }

      pivotMotor.set(pidPower);
    }

    Logger.recordOutput("Ground-Intake/Distance-Sensor/Distance", getDistanceSensor());
    // Logger.recordOutput("Ground-Intake/Distance-Sensor/Sdev", sensor_sdev);
    // Logger.recordOutput("Ground-Intake/Distance-Sensor/Status", m_rangeSensor.getStatus());
    Logger.recordOutput("Ground-Intake/Distance-Sensor/Using-Distance-Sensor", wanttoPOP);
    Logger.recordOutput("Ground-Intake/Distance-Sensor/Tripped", getDistanceSensorTripped());

    Logger.recordOutput("Ground-Intake/Pivot-Position", getPivotPosition());
    // Logger.recordOutput("Grount-Intake/PID-Power", motorpower);
    Logger.recordOutput(
        "Ground-Intake/Pivot-Abs-Position", pivotMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Ground-Intake/Pivot-Target-Position", targetPosition);
    Logger.recordOutput("Ground-Intake/Pivot-Power", pivotMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Pivot-Current", pivotMotor.getOutputCurrent());

    Logger.recordOutput("Ground-Intake/Front-Power", frontIntakeMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Front-Current", frontIntakeMotor.getOutputCurrent());

    Logger.recordOutput("Ground-Intake/Back-Power", backIntakeMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Back-Current", backIntakeMotor.getOutputCurrent());

    if (targetPosition == Constants.Presets.groundIntakeIntake
        && getDistanceSensorTripped()
        && wanttoPOP) {
      setIntakePosition(Constants.Presets.groundIntakePop);
    }
  }

  public void setFastPID(boolean fastPID) {
    this.fastPID = fastPID;
  }

  public double getDistanceSensor() {
    return m_rangeSensor.getRange();
  }

  public boolean getDistanceSensorTripped() {
    return getDistanceSensor() <= Constants.GroundIntake.CoralDistanceTheshold;
  }

  public void setIntakeCUrrentlim(int current) {
    SparkFlexConfig pivotMotorConfig = new SparkFlexConfig();
    pivotMotorConfig.smartCurrentLimit(current);
    pivotMotor.configure(
        pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
