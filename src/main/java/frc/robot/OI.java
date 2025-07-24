package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  public static XboxController primaryController =
      new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);
  public static Joystick rightButtonBoard = new Joystick(2);

  public static Joystick leftButtonBoard = new Joystick(1);

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

  public static double getIntakePower() {

    double Sean_intake_power =
        deadband(-primaryController.getRightTriggerAxis(), Constants.Intake.DEADBAND)
            + deadband(primaryController.getLeftTriggerAxis(), Constants.Intake.DEADBAND)+ (primaryController.getAButton() ? -0.5: 0);
    double Ashton_intake_power =
        deadband(rightButtonBoard.getRawAxis(0), Constants.Intake.DEADBAND);
    return Sean_intake_power + Ashton_intake_power;
  }

  public static BooleanEvent activateGroundIntake(){
    // double Sean_intake_power =
    //     deadband(-primaryController.getRightTriggerAxis(), Constants.Intake.DEADBAND)
    //         + deadband(primaryController.getLeftTriggerAxis(), Constants.Intake.DEADBAND);
    // return new BooleanEvent(eventLoop, ()->(Sean_intake_power<0));
    return primaryController.rightTrigger(0.05, eventLoop);
    // return primaryController.a(eventLoop);
  }
  public static BooleanEvent L1GroundIntake(){
    return rightButtonBoard.button(8, eventLoop).or(rightButtonBoard.button(7, eventLoop));
  }
  public static BooleanEvent L1Outtake(){
    return leftButtonBoard.button(9, eventLoop);
  }
  public static BooleanEvent groundIntakeIntake(){
    return rightButtonBoard.button(9, eventLoop);
  }
  public static BooleanEvent groundIntakeOuttake(){
    return rightButtonBoard.button(10, eventLoop);
  }
  

  public static double getElevatorPower() {
    // TODO: assign a button
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? -0.5 : 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? 0.5 : 0.0);
    return upPower + downPower;
  }

  public static double getArmPower() {
    double leftPower = (primaryController.povLeft(eventLoop).getAsBoolean() ? 1 : 0.0);
    double rightPower = (primaryController.povRight(eventLoop).getAsBoolean() ? -1 : 0.0);
    return leftPower + rightPower;
  }

  public static double getClimberPower() {
    double upPower = (primaryController.povUp(eventLoop).getAsBoolean() ? 0.5 : 0.0);
    double downPower = (primaryController.povDown(eventLoop).getAsBoolean() ? -0.25 : 0.0);
    return upPower + downPower;
  }

  public static BooleanEvent bargeYeet() {
    return leftButtonBoard.button(7, eventLoop);
  }
  public static BooleanEvent bargeStage() {
    return leftButtonBoard.button(9, eventLoop);
  }

  public static BooleanEvent lowIntake() {
    return leftButtonBoard.button(10, eventLoop);
  }

  public static BooleanEvent lowLowIntake() {
    return rightButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent lolipop() {
    return leftButtonBoard.button(3, eventLoop);
  }

  public static BooleanEvent Intake() {
    return leftButtonBoard.button(6, eventLoop);
  }

  public static BooleanEvent Processor() {
    return leftButtonBoard.button(8, eventLoop);
  }

  public static BooleanEvent L3Algea() {
    return leftButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent L3Coral() {
    return leftButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent L2Algea() {
    return leftButtonBoard.button(4, eventLoop);
  }

  public static BooleanEvent L2Coral() {
    return leftButtonBoard.button(5, eventLoop);
  }

  public static BooleanEvent startClimb() {
    return rightButtonBoard.button(1, eventLoop);
  }

  public static BooleanEvent ClimbStage0() {
    return rightButtonBoard.button(5, eventLoop);
  }
  public static BooleanEvent ToggleDefenseMode() {
    return rightButtonBoard.button(5, eventLoop);
  }

  public static BooleanEvent ClimbStage1() {
    return rightButtonBoard.button(2, eventLoop);
  }

  public static BooleanEvent ClimbStage2() {
    return rightButtonBoard.button(3, eventLoop);
  }

  public static BooleanEvent l1Test() {
    return leftButtonBoard.button(10, eventLoop);
  }

  public static double powerCurved(double inputPower) {
    double newValue = Math.pow(inputPower, 2);
    if (inputPower < 0) return -newValue;
    return newValue;
  }
}
