package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class OI {
  // * Link to Control Scheme:
  // *https://docs.google.com/presentation/d/1tF5M_lSn-6BPPR1MytJeh-sMJD9G1pwS/edit?usp=sharing&ouid=112393567782371061757&rtpof=true&sd=true

  private static EventLoop eventLoop = new EventLoop();
  public static XboxController primaryController =
      new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);
  public static Joystick leftButtonBoard = new Joystick(1);
  public static Joystick rightButtonBoard = new Joystick(2);

  public static XboxController getPrimaryController() {
    return primaryController;
  }

  public static void update() {
    eventLoop.poll();
  }

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) > deadband) {
      if (input > 0.0) {
        return (input - deadband) / (1.0 - deadband);
      }

      return (input + deadband) / (1.0 - deadband);
    }

    return 0.0;
  }

  public static double getIntakePower(boolean coralIntendedForL1) {
    double Sean_intake_power =
        deadband(-primaryController.getRightTriggerAxis(), Constants.Intake.DEADBAND)
            + (!coralIntendedForL1
                ? OI.deadband(OI.controllerOuttake(), Constants.Intake.DEADBAND)
                : 0);
    double Ashton_intake_power =
        deadband(rightButtonBoard.getRawAxis(0), Constants.Intake.DEADBAND);
    return Sean_intake_power + Ashton_intake_power;
  }

  public static double controllerOuttake() {
    return primaryController.getLeftTriggerAxis();
  }

  public static BooleanEvent activateGroundIntake() {
    // double Sean_intake_power =
    //     deadband(-primaryController.getRightTriggerAxis(), Constants.Intake.DEADBAND)
    //         + deadband(primaryController.getLeftTriggerAxis(), Constants.Intake.DEADBAND);
    // return new BooleanEvent(eventLoop, ()->(Sean_intake_power<0));
    return primaryController.rightTrigger(0.05, eventLoop);
    // return primaryController.a(eventLoop);
  }

  public static BooleanEvent L1GroundIntake() {
    return (primaryController.a(eventLoop))
        .or(
            () -> {
              if ((rightButtonBoard.getRawAxis(1)) > Constants.Intake.DEADBAND) {
                return true;
              }
              return false;
            });
  }

  public static BooleanEvent toggleDistanceSensor() {
    return rightButtonBoard.button(7, eventLoop);
  }

  public static BooleanEvent groundIntakeIntake() {
    // ! Disabled
    return new BooleanEvent(eventLoop, () -> false);
    // return rightButtonBoard.button(9, eventLoop);
  }

  public static BooleanEvent groundIntakeOuttake() {
    return rightButtonBoard.button(10, eventLoop);
  }

  public static BooleanEvent groundIntakeManualOut() {
    return rightButtonBoard
        .button(8, eventLoop)
        .or(
            () -> {
              return (rightButtonBoard.getRawAxis(1)) < -Constants.Intake.DEADBAND;
            });
  }

  public static BooleanEvent groundHover() {
    return leftButtonBoard.button(10, eventLoop);
  }

  public static double getElevatorManualPower() {
    // double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? -0.5 : 0.0);
    // double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? 0.5 : 0.0);
    return deadband(-leftButtonBoard.getRawAxis(1), 0.1);
  }

  public static BooleanEvent rezeroElevator() {
    return leftButtonBoard.button(3, eventLoop);
  }

  // public static double getArmPower() {
  //   double leftPower = (primaryController.povLeft(eventLoop).getAsBoolean() ? 1 : 0.0);
  //   double rightPower = (primaryController.povRight(eventLoop).getAsBoolean() ? -1 : 0.0);
  //   return leftPower + rightPower;
  // }

  // public static double getClimberPower() {
  //   double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? 0.5 : 0.0);
  //   double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? -0.25 : 0.0);
  //   return upPower + downPower;
  // }

  public static BooleanEvent bargeYeet() {
    return leftButtonBoard.button(7, eventLoop);
  }

  public static BooleanEvent bargeStage() {
    return leftButtonBoard.button(8, eventLoop);
  }

  public static BooleanEvent armSafeMode() {
    return rightButtonBoard.button(6, eventLoop);
  }

  public static BooleanEvent groundIntakeUp() {
    return rightButtonBoard.button(4, eventLoop).or(primaryController.povUp(eventLoop));
  }

  public static BooleanEvent lolipop() {
    // ! Disabled
    return new BooleanEvent(eventLoop, () -> false);
    // return leftButtonBoard.button(3, eventLoop);
  }

  public static BooleanEvent feeder() {
    // ! Disabled
    return new BooleanEvent(eventLoop, () -> false);
    // return leftButtonBoard.button(6, eventLoop);
  }

  public static BooleanEvent Processor() {
    return leftButtonBoard.button(9, eventLoop);
  }

  public static BooleanEvent L3Algea() {
    return leftButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent L3Coral() {
    return leftButtonBoard.button(2, eventLoop).or(primaryController.x(eventLoop));
  }

  public static BooleanEvent L2Algea() {
    return leftButtonBoard.button(4, eventLoop);
  }

  public static BooleanEvent L2Coral() {
    return leftButtonBoard.button(5, eventLoop).or(primaryController.b(eventLoop));
  }

  public static BooleanEvent startClimb() {
    return rightButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent ToggleDefenseMode() {
    return rightButtonBoard.button(5, eventLoop).or(primaryController.povDown(eventLoop));
  }

  public static BooleanEvent ClimbStage1() {
    return rightButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent ClimbStage2() {
    return rightButtonBoard.button(3, eventLoop);
  }

  public static BooleanEvent climberHover() {
    return rightButtonBoard.button(9, eventLoop);
  }

  public static BooleanEvent l1Test() {
    // ! Disabled
    return new BooleanEvent(eventLoop, () -> false);
    // return leftButtonBoard.button(10, eventLoop);
  }

  public static double powerCurved(double inputPower) {
    double newValue = Math.pow(inputPower, 2);
    if (inputPower < 0) return -newValue;
    return newValue;
  }
}
