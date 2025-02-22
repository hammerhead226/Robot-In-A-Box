package frc.robot.subsystems.scoral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import org.littletonrobotics.junction.Logger;

public class ScoralArmIOTalonFX implements ArmIO {
  private final TalonFX leader;
  private final CANcoder scoralCoder;
  private double positionSetpointDegs;

  private double startAngleDegs;

  private StatusSignal<Angle> leaderPositionRotations;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  public ScoralArmIOTalonFX(int leadID, int canCoderID) {
    CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // change?
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // OFFSET IS IN ROTATIONS
    // coderConfig.MagnetSensor.withMagnetOffset(Units.degreesToRotations(58));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.CoralScorerConstants.ScoralArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.CoralScorerConstants.ScoralArmConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.Feedback.FeedbackRemoteSensorID = canCoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // config.Feedback.SensorToMechanismRatio = 1;
    config.Feedback.SensorToMechanismRatio = 1;
    // SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    // config.Feedback.RotorToSensorRatio =
    //     SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    leader = new TalonFX(leadID);
    scoralCoder = new CANcoder(canCoderID);

    leader.getConfigurator().apply(config);
    scoralCoder.getConfigurator().apply(coderConfig);
    leader.setPosition(
        (scoralCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(57 - 12))
            * SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    // leader.setPosition(1 * 360.);
    // leader.setPosition(
    //     Units.degreesToRotations(scoralCoder.getAbsolutePosition().getValueAsDouble())
    //         / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    // Conversions.degreesToFalcon(
    //     Units.rotationsToDegrees(scoralCoder.getAbsolutePosition().getValueAsDouble()),
    //     SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO));

    leaderPositionRotations = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    // leader.get

    positionSetpointDegs =
        SubsystemConstants.CoralScorerConstants.ScoralArmConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, leaderPositionRotations, velocityDegsPerSec, appliedVolts, currentAmps);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionRotations, velocityDegsPerSec, appliedVolts, currentAmps);
    Logger.recordOutput("scoral arm motor rotations", leaderPositionRotations.getValueAsDouble());
    Logger.recordOutput(
        "scoral arm motor setpoint",
        Units.degreesToRotations(this.positionSetpointDegs)
            * SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    inputs.positionDegs =
        Units.rotationsToDegrees(leaderPositionRotations.getValueAsDouble())
            / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    // inputs.positionDegs = Units.rotationsToDegrees(leaderPositionRotations.getValueAsDouble());
    // * 360.
    // / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    // * 360.
    // / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    // Units.rotationsToDegrees(
    //     leaderPositionRotations.getValueAsDouble()
    //         / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    // / SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO;
    // inputs.positionDegs =
    //     Conversions.falconToDegrees(
    //             (leaderPositionDegs.getValueAsDouble()),
    //             SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_GEAR_RATIO)
    //         + SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_ZERO_ANGLE;

    inputs.velocityDegsPerSec = Units.rotationsToDegrees(velocityDegsPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionSetpointDegs = positionSetpointDegs;
    Logger.recordOutput(
        "cancoder arm position degs",
        Units.rotationsToDegrees(
            scoralCoder.getAbsolutePosition().getValueAsDouble()
                - Units.degreesToRotations(57 - 12)));
  }

  @Override
  public void setBrakeMode(boolean bool) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (bool) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    leader.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;

    // leader.setVoltage(1);
    leader.setControl(
        new PositionVoltage(
                Units.degreesToRotations(positionDegs)
                    * SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_GEAR_RATIO)
            .withFeedForward(ffVolts)); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    this.positionSetpointDegs = leaderPositionRotations.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
  }
}
