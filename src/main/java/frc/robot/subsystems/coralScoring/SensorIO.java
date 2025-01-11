package frc.robot.subsystems.coralScoring;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
    @AutoLog
    public static class sensorIOInputs {
        public double distance = 0;
        public boolean connected = false;  
    }

    public default void updateInputs(sensorIOInputs inputs) {}

}
