package frc.robot.subsystems.motor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.commoniolayers.MotorIO;
import frc.robot.subsystems.commoniolayers.MotorIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Motor extends SubsystemBase {
  private final MotorIO motor;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 1);

  /** Creates a new Flywheel. */
  public Motor(MotorIO motor) {
    this.motor = motor;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        motor.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.1);
        motor.configurePID(0, 0.0, 0.0);
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
    motor.updateInputs(inputs);
    Logger.processInputs("Test Motor", inputs);
    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    motor.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Debug Rollers/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {
    // Elastic.sendNotification(
    // new Notification(
    // NotificationLevel.INFO, "Notice", "Flywheel is being run at " + volts + "
    // volts."));
    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this).withTimeout(5);
  }

  /** Stops the flywheel. */
  public void stop() {
    motor.stop();
  }

  public Command motorStop() {

    return new InstantCommand(() -> stop(), this);
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
  @AutoLogOutput(key = "Debug Rollers/Velocity RPM")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void zero() {
    motor.stop();
  }

  private void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }
  }
}
