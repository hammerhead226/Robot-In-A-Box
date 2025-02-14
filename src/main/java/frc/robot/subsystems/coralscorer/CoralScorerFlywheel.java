package frc.robot.subsystems.coralscorer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.algae.FeederIOInputsAutoLogged;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.commoniolayers.FlywheelIOInputsAutoLogged;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralScorerFlywheel extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final CoralSensorIO sensor;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final CoralSensorIOInputsAutoLogged sInputs = new CoralSensorIOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private AlgaeState lastAlgaeState;
  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 1);

  public enum ScoralFlywheelState {
    ZERO,
    INTAKING_CORAL,
    INTAKING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE
  }

  public ScoralFlywheelState currentState = ScoralFlywheelState.ZERO;
  public ScoralFlywheelState wantedState = ScoralFlywheelState.ZERO;

  private CoralState lastCoralState;

  /** Creates a new Flywheel. */
  public CoralScorerFlywheel(
      FlywheelIO flywheel,
      CoralSensorIO sensor,
      CoralState lastCoralState,
      AlgaeState lastAlgaeState) {
    this.flywheel = flywheel;
    this.sensor = sensor;
    this.lastCoralState = lastCoralState;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        flywheel.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.1);
        flywheel.configurePID(0, 0.0, 0.0);
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
    flywheel.updateInputs(inputs);
    Logger.processInputs(" ballsFlywheel", inputs);

    if (wantedState != currentState) {
      currentState = wantedState;
    }

    switch (currentState) {
      case ZERO:
        zero();
        break;
      case INTAKING_ALGAE:
        intakeAlgae();
        break;
      case INTAKING_CORAL:
        intakeCoral();
        break;
      case SCORING_ALGAE:
        scoreAlgae();
        break;
      case SCORING_CORAL:
        scoreCoral();
        break;

      default:
        zero();
    }

    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    flywheel.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    flywheel.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {
    Elastic.sendNotification(
        new Notification(
            NotificationLevel.INFO, "Notice", "Flywheel is being run at " + volts + " volts."));
    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
  public void stop() {
    flywheel.stop();
  }

  public Command stopCommand() {

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

  public AlgaeState seesAlgae() {
    Logger.recordOutput("see note val", "default");
    if (feedInputs.currentAmps > 13) {
      Logger.recordOutput("see note val", "current");
      lastAlgaeState = AlgaeState.CURRENT;
      return AlgaeState.CURRENT;

    } else {
      Logger.recordOutput("see note val", "no note");
      lastAlgaeState = AlgaeState.NO_ALGAE;
      return AlgaeState.NO_ALGAE;
    }
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

  public CoralState seesCoral() {
    Logger.recordOutput("see note val", "default");
    if ((sInputs.distance < SubsystemConstants.CORAL_DIST)) {
      Logger.recordOutput("see note val", "sensor");
      lastCoralState = CoralState.SENSOR;
      return CoralState.SENSOR;

    } else if (feedInputs.currentAmps > 1399999999) {
      Logger.recordOutput("see note val", "current");
      lastCoralState = CoralState.CURRENT;
      return CoralState.CURRENT;

    } else {
      Logger.recordOutput("see note val", "no note");
      lastCoralState = CoralState.NO_CORAL;
      return CoralState.NO_CORAL;
    }
  }

  public CoralState getLastCoralState() {
    return lastCoralState;
  }

  public void setWantedState(ScoralFlywheelState state) {
    wantedState = state;
  }

  public void zero() {
    flywheel.stop();
  }

  public void intakeCoral() {
    flywheel.setVoltage(-2);
  }

  public void intakeAlgae() {
    flywheel.setVoltage(-1);
  }

  public void scoreCoral() {
    flywheel.setVoltage(2);
  }

  public void scoreAlgae() {
    flywheel.setVoltage(2);
  }

  private void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }
  }
}
