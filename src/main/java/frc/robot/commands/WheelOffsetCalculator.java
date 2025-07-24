package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.util.ArrayList;

//
public class WheelOffsetCalculator extends Command {
  private final Drive drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheelOffsetCalculator(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    ArrayList<Rotation2d> positions = drive.getRawTurnEncoderPositions();

    if (positions == null || positions.size() < 4) {
      System.err.println("Error: Encoder positions are invalid.");
      return;
    }

    DecimalFormat df = new DecimalFormat("#.#");

    String PURPLE = "";
    String ANSI_RESET = "";

    System.out.println(
        PURPLE
            + "public static final double FRONT_LEFT_OFFSET = Math.toRadians("
            + df.format(positions.get(0).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double FRONT_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions.get(1).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double BACK_LEFT_OFFSET = Math.toRadians("
            + df.format(positions.get(2).getDegrees())
            + ");"
            + ANSI_RESET);
    System.out.println(
        PURPLE
            + "public static final double BACK_RIGHT_OFFSET = Math.toRadians("
            + df.format(positions.get(3).getDegrees())
            + ");"
            + ANSI_RESET);

    System.out.println("\n");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
