package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public static class WinchIOInputs {
    public double winchVelocityRPM = 0;
    public double winchPositionRads;
    public double statorCurrentAmps = 0;
    public double supplyCurrentAmps = 0;
    public double appliedVolts = 0;
    public double velocitySetpointRPM = 0;
    public double velocityRadPerSec = 0;
    public double positionRad;
  }

  public default void updateInputs(WinchIOInputs inputs) {}

  public default void setVelocityRPM(double velocityRPS, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}
}
