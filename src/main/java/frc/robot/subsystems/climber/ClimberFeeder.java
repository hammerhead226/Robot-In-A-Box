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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberFeeder extends SubsystemBase {
  private final ClimberFeederIO io;
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  // private final AlgaeState algaeState;

  private AlgaeState lastAlgaeState;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 1);

  // private final DistanceSensorIOInputsAutoLogged sInputs = new
  // DistanceSensorIOInputsAutoLogged();
  /** Creates a new Flywheel. */
  public ClimberFeeder(ClimberFeederIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
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
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    updateTunableNumbers();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityRPS(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {

    return new InstantCommand(() -> runVolts(volts), this).withTimeout(5);
  }

  public AlgaeState getAlgaeState() {
    return lastAlgaeState;
  }

  public AlgaeState seesAlgae() {
    Logger.recordOutput("see coral val", "default");
    if (inputs.currentAmps > 13) { // TODO add additional check to filter out false positives
      // } else if (feedInputs.currentAmps > 10000) {
      Logger.recordOutput("see coral val", "current");
      lastAlgaeState = AlgaeState.CURRENT;
      return AlgaeState.CURRENT;

    } else {
      Logger.recordOutput("see coral val", "no coral");
      lastAlgaeState = AlgaeState.NO_ALGAE;
      return AlgaeState.NO_ALGAE;
    }
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
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
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  private void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get(), 1);
    }
  }
}
