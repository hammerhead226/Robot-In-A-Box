package frc.robot.subsystems.scoral;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceSensorIO {
  @AutoLog
  public static class DistanceSensorIOInputs {
    public double distanceInches = 0; // assuming distance or proximity sensor
    public boolean connected = false;
    public double signalStrength = 0;
  }

  public default void updateInputs(DistanceSensorIOInputs inputs) {}
}
