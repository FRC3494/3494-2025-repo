package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

public class ClimberIO {
  SparkMax climberMotor;

  public static enum ClimberMode {
    Manual,
    Automatic
  }

  @AutoLog
  public static class ClimberIOInputs {
    double targetPosition;
    double power;
    double climberPosition;

    ClimberMode mode;
  }

  public ClimberIO(SparkMax climberMotor) {
    this.climberMotor = climberMotor;
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPosition = climberMotor.getEncoder().getPosition();
  }
}
