package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberFeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederVelocityRPM = 0;
    public double feederRotations;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double velocitySetpointRPM = 0;
    public double velocityRadPerSec = 0;
    public double positionRad;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVelocityRPS(double velocityRPS, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}
}
