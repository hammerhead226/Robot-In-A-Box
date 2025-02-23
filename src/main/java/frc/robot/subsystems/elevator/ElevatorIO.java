package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    double positionInch = 0;
    double positionSetpointInch = 0;
    double elevatorVelocityInchesPerSecond = 0;
    double currentAmps = 0;
    double appliedVolts = 0;
    double CANrangeDistanceInches;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runCharacterization(double volts) {}

  public default void setPositionSetpoint(double position, double ffVolts) {}

  public default void stop() {}

  public default void setVoltage(double volts) {}

  public default void configurePIDF(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {}

  public default void setBrakeMode(boolean brake) {}
}
