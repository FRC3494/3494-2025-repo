package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {
  private SparkFlex pivotMotor;
  private SparkFlexConfig pivotMotorConfig;

  private SparkMax frontIntakeMotor;
  private SparkMaxConfig frontIntakeMotorConfig;
  private SparkMax backIntakeMotor;
  private SparkMaxConfig backIntakeMotorConfig;
  public boolean wanttoPOP = true;
  private PIDController pivotMotorPIDLoop = new PIDController(8, 0, 0);
  public double targetPosition;
  public double hoverPosition = Constants.Presets.groundIntakeHover;

  // Create instance of Time-Of_Flight driver for device 1
  private final TimeOfFlight m_rangeSensor =
      new TimeOfFlight(Constants.GroundIntake.distanceSensorDeviceNumber);

  public GroundIntake() {
    pivotMotor = new SparkFlex(Constants.GroundIntake.pivotMotor, MotorType.kBrushless);
    frontIntakeMotor = new SparkMax(Constants.GroundIntake.frontIntakeMotor, MotorType.kBrushless);
    backIntakeMotor = new SparkMax(Constants.GroundIntake.backIntakeMotor, MotorType.kBrushless);

    targetPosition = pivotMotor.getAbsoluteEncoder().getPosition();
    pivotMotorConfig = new SparkFlexConfig();
    frontIntakeMotorConfig = new SparkMaxConfig();
    backIntakeMotorConfig = new SparkMaxConfig();

    pivotMotorConfig.smartCurrentLimit(45);
    pivotMotorConfig.closedLoop.pidf(8.0, 0, 0, 0.4);
    pivotMotorConfig.closedLoop.outputRange(-0.9, 0.9); // 0.8// 0.7

    pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(6.0);

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
    pivotMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    targetPosition = position;
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

    double sensor_distance = m_rangeSensor.getRange();
    double sensor_sdev = m_rangeSensor.getRangeSigma();
    Logger.recordOutput("Ground-Intake/Distance-Sensor/Distance", sensor_distance);
    // Logger.recordOutput("Ground-Intake/Distance-Sensor/Sdev", sensor_sdev);
    // Logger.recordOutput("Ground-Intake/Distance-Sensor/Status", m_rangeSensor.getStatus());
    Logger.recordOutput("Ground-Intake/Using-Distance-Sensor",wanttoPOP);

    Logger.recordOutput("Ground-Intake/Pivot-Position", pivotMotor.getEncoder().getPosition());
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
        && sensor_distance < Constants.GroundIntake.CoralDistanceTheshold && wanttoPOP) {
      setIntakePosition(Constants.Presets.groundIntakePop);
    }
  }

  public double getDistanceSensor() {
    return m_rangeSensor.getRange();
  }
  public void setIntakeCUrrentlim(int current){
    pivotMotorConfig.smartCurrentLimit(current);
    pivotMotor.configure(
        pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
