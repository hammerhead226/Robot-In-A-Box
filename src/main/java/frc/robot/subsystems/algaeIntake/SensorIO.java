package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
    @AutoLog
    public static class SensorIOInputs {
        public double distance = 0;
        public boolean connected = false;  
    }

    public default void updateInputs(SensorIOInputs inputs) {}

}
