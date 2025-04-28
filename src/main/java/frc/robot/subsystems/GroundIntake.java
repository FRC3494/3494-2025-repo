package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.google.googlejavaformat.Indent.Const;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase  {
    private SparkFlex pivotMotor;
    private SparkFlexConfig pivotMotorConfig;

    private SparkMax frontIntakeMotor;
    private SparkMaxConfig frontIntakeMotorConfig;
    private SparkMax backIntakeMotor;
    private SparkMaxConfig backIntakeMotorConfig;

    public double targetPosition = 99999.0;
    public double hoverPosition = Constants.Presets.groundIntakeHover;
   

    public GroundIntake(){
        pivotMotor = new SparkFlex(Constants.GroundIntake.pivotMotor, MotorType.kBrushless);
        frontIntakeMotor = new SparkMax(Constants.GroundIntake.frontIntakeMotor, MotorType.kBrushless);
        backIntakeMotor = new SparkMax(Constants.GroundIntake.backIntakeMotor, MotorType.kBrushless);

        pivotMotorConfig = new SparkFlexConfig();
        frontIntakeMotorConfig = new SparkMaxConfig();
        backIntakeMotorConfig = new SparkMaxConfig();

        pivotMotorConfig.smartCurrentLimit(45);
        pivotMotorConfig.closedLoop.pid(8, 0, 0);
        pivotMotorConfig.closedLoop.outputRange(-0.6, 0.6);
        pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
        pivotMotorConfig.idleMode(IdleMode.kBrake);

        frontIntakeMotorConfig.idleMode(IdleMode.kBrake);
        frontIntakeMotorConfig.smartCurrentLimit(30);
        frontIntakeMotorConfig.inverted(false);

        backIntakeMotorConfig.idleMode(IdleMode.kBrake);
        backIntakeMotorConfig.smartCurrentLimit(30);
        backIntakeMotorConfig.inverted(false);

        pivotMotor.configure(
            pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        frontIntakeMotor.configure(
            frontIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backIntakeMotor.configure(
                backIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakePosition(double position) {
        pivotMotor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
        targetPosition = position;
    }

    public void setIntakePower(double front, double back){
        frontIntakeMotor.set(front);
        backIntakeMotor.set(back);
    }

    @Override
  public void periodic() {
    Logger.recordOutput("Ground-Intake/Pivot-Position", pivotMotor.getEncoder().getPosition());
    Logger.recordOutput("Ground-Intake/Pivot-Abs-Position", pivotMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Ground-Intake/Pivot-Target-Position", targetPosition);
    Logger.recordOutput("Ground-Intake/Pivot-Power", pivotMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Pivot-Current", pivotMotor.getOutputCurrent());

    Logger.recordOutput("Ground-Intake/Front-Power", frontIntakeMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Front-Current", frontIntakeMotor.getOutputCurrent());

    Logger.recordOutput("Ground-Intake/Back-Power", backIntakeMotor.getAppliedOutput());
    Logger.recordOutput("Ground-Intake/Back-Current", backIntakeMotor.getOutputCurrent());


  }
}
