package frc.robot.subsystems.scoral;

import org.littletonrobotics.junction.AutoLog;

public interface ScoralSensorIO {
  @AutoLog
  public static class ScoralSensorIOInputs {
    public double distanceInches = 0; // assuming distance or proximity sensor
    public boolean connected = false;
  }

  public default void updateInputs(ScoralSensorIOInputs inputs) {}
}
