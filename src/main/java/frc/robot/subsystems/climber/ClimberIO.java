package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
  SparkMax climberMotor;

  @AutoLog
  public static class ClimberIOInputs {
    double targetPosition;
    double power;
    double climberPosition;
  }

  public ClimberIO(SparkMax climberMotor) {
    this.climberMotor = climberMotor;
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPosition = climberMotor.getEncoder().getPosition();
  }
}
