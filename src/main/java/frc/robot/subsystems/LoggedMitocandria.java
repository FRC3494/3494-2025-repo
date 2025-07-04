package frc.robot.subsystems;

import au.grapplerobotics.CouldNotGetException;
import au.grapplerobotics.MitoCANdria;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LoggedMitocandria {
  MitoCANdria mito;

  public LoggedMitocandria() {
    mito = new MitoCANdria(Constants.Mitocandria.MITOCANDRIA_CAN_ID);

    try {
      mito.setChannelEnabled(Constants.Mitocandria.LIMELIGHT_CHANNEL, true);
      mito.setChannelEnabled(Constants.Mitocandria.PIGEON_CHANNEL, true);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void periodic() {
    logChannel("Limelight", Constants.Mitocandria.LIMELIGHT_CHANNEL);
    logChannel("Pigeon", Constants.Mitocandria.PIGEON_CHANNEL);
  }

  private void logChannel(String channelName, int channelInteger) {
    try {
      mito.getChannelEnabled(channelInteger)
          .ifPresent(
              (enabled) ->
                  Logger.recordOutput("Mitocandira/" + channelName + "Channel/Enabled", enabled));
      mito.getChannelVoltage(channelInteger)
          .ifPresent(
              (voltage) ->
                  Logger.recordOutput("Mitocandira/" + channelName + "Channel/Voltage", voltage));
      mito.getChannelCurrent(channelInteger)
          .ifPresent(
              (current) ->
                  Logger.recordOutput("Mitocandira/" + channelName + "Channel/Current", current));
    } catch (CouldNotGetException e) {
      e.printStackTrace();
    }
  }
}
