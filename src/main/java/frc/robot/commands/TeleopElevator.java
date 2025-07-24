package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperStructure.Elevator;
import org.littletonrobotics.junction.Logger;

public class TeleopElevator extends Command {
  Elevator elevator;
  private double elevatorPower = 0;

  public TeleopElevator(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    // elevatorPower = OI.deadband(OI.getElevatorPower(), 0.05);
    double elevatorPower = -RobotContainer.leftButtonBoard.getRawAxis(1);
    elevatorPower = OI.deadband(elevatorPower, 0.1);
    Logger.recordOutput("Elevator/Manual-Power", elevatorPower);
    if (elevatorPower != 0) {
      elevator.setElevatorPosition(elevator.getTicks() + elevatorPower);
    }
  }
}
