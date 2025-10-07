// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.commoniolayers;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public __________ void updateInputs(MotorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Write a method stop() */
  ____________________

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
