package frc.robot.subsystems.commoniolayers;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerPositionInch = 0;
    public double indexerVelocityInchesPerSecond = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double positionSetpointInch = 0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runCharacterization(double volts) {}

  default void stop() {}

  default void setPositionSetpoint(double positionDegs, double ffvolts) {}

  default void configurePID(double kP, double kI, double kD) {}
}
