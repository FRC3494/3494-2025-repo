package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.LEDPattern;

public class LEDs extends SubsystemBase {
  Spark led;
  LEDPattern pattern;

  public LEDs() {
    led = new Spark(Constants.LEDs.LED_PWM_ID);
    pattern = LEDPattern.NONE;
    led.set(pattern.value);
  }

  public Command setPattern(LEDPattern pattern) {
    return new InstantCommand(
            () -> {
              if (this.pattern != pattern) {
                this.pattern = pattern;
                led.set(pattern.value);
              }
            })
        .ignoringDisable(true);
  }

  public LEDPattern getPattern() {
    return pattern;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDs/Pattern", pattern.name());
  }
}
