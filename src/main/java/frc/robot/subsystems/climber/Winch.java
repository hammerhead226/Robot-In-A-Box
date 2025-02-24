// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Winch extends SubsystemBase {
  private final WinchIO io;
  private final WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Winch/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Winch/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Winch/kA", 1);

  // private final DistanceSensorIOInputsAutoLogged sInputs = new
  // DistanceSensorIOInputsAutoLogged();
  /** Creates a new Flywheel. */
  public Winch(WinchIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.01);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.01);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.3);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Winch Debug/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    updateTunableNumbers();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Winch", inputs);
    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityRPM(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Winch Debug/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {

    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command stopWinch() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the Winch. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  // @AutoLogOutput
  // public double getVelocityRPM() {
  //   return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  // }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(inputs.winchVelocityRPM);
  }

  private void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get(), 1);
    }
  }
}
