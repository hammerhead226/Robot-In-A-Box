package frc.robot.subsystems.coralscorer;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  public static class CoralSensorIOInputs {
    public double distanceInches = 0; // assuming distance or proximity sensor
    public boolean connected = false;
  }

  public default void updateInputs(CoralSensorIOInputs inputs) {}
}
