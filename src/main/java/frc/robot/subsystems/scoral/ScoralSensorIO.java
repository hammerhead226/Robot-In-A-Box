package frc.robot.subsystems.scoral;

import org.littletonrobotics.junction.AutoLog;

public interface ScoralSensorIO {
  @AutoLog
  public static class CoralSensorIOInputs {
    public double distanceInches = 0; // assuming distance or proximity sensor
    public boolean connected = false;
  }

  public default void updateInputs(CoralSensorIOInputs inputs) {}
}
