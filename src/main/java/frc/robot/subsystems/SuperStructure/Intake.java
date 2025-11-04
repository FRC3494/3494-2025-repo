package frc.robot.subsystems.SuperStructure;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;
  private double intakeSpeed;

  boolean hasCoral = false;
  private double currentSpeed = 0;
  ArrayList<Double> currents = new ArrayList<>();

  public Intake() {
    Logger.recordOutput("Intake/Intake-Power", intakeSpeed);
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(30); // was 40 on 7/9/25
    intakeConfig.inverted(false);
    intakeMotor = new SparkMax(Constants.Intake.intakeMotor, MotorType.kBrushless);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    currentSpeed = speed;
    intakeMotor.set(speed);
  }

  public double currentAverage(double currentCurrent) {
    currents.add(0, currentCurrent);

    if (currents.size() >= 10) {
      currents.remove(currents.size() - 1);
    }
    double average = 0;
    for (int i = 0; i < currents.size(); i++) {
      average += (double) currents.get(i);
    }
    average /= currents.size();
    return average;
  }

  @Override
  public void periodic() {
    double Cavrg = currentAverage(intakeMotor.getOutputCurrent());
    if (Cavrg > 13) {
      hasCoral = true;
    }
    Logger.recordOutput("Intake/Current-Average", Cavrg);
    Logger.recordOutput("Intake/Has-Coral", hasCoral);
    Logger.recordOutput("Intake/Current-Speed", currentSpeed);
    Logger.recordOutput("Intake/MotorTemp", intakeMotor.getMotorTemperature());
    if (currentSpeed > 0.05) {
      hasCoral = false;
    }
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void updateIdleMode() {
    SparkMaxConfig config = new SparkMaxConfig();

    switch (Constants.DRIVE_MODE) {
      case DEMO, DEMO_AUTOALIGN -> {
        config.idleMode(IdleMode.kCoast);
      }
      default -> {
        config.idleMode(IdleMode.kBrake);
      }
    }

    intakeMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
