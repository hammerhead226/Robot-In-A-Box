package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final CANrange distanceSensor;

  private double positionSetpoint;
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> leaderStatorCurrentAmps;
  private final StatusSignal<Current> leaderSupplyCurrentAmps;
  private final StatusSignal<Current> followerStatorCurrentAmps;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Distance> canRangeDistance;

  public ElevatorIOTalonFX(int lead, int follow, int canRangeID) {
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    leadConfig.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT;
    leadConfig.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leader = new TalonFX(lead, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(follow, SubsystemConstants.CANIVORE_ID_STRING);
    distanceSensor = new CANrange(canRangeID, SubsystemConstants.CANIVORE_ID_STRING);

    leader.getConfigurator().apply(leadConfig);

    positionSetpoint = SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;

    follower.setControl(new Follower(lead, true));

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    follower.getConfigurator().apply(followerConfig);

    elevatorPosition = leader.getPosition();
    elevatorVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    leaderStatorCurrentAmps = leader.getStatorCurrent();
    leaderSupplyCurrentAmps = leader.getSupplyCurrent();
    followerStatorCurrentAmps = follower.getStatorCurrent();
    followerSupplyCurrentAmps = follower.getSupplyCurrent();
    canRangeDistance = distanceSensor.getDistance();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        canRangeDistance,
        elevatorPosition,
        elevatorVelocity,
        appliedVolts,
        leaderStatorCurrentAmps,
        leaderSupplyCurrentAmps,
        followerStatorCurrentAmps,
        followerSupplyCurrentAmps);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        canRangeDistance,
        elevatorPosition,
        elevatorVelocity,
        appliedVolts,
        leaderStatorCurrentAmps,
        leaderSupplyCurrentAmps,
        followerStatorCurrentAmps,
        followerSupplyCurrentAmps);

    // Logger.recordOutput(
    //     "Debug Elevator/Elevator CANRage Distance Inch",
    //     distanceSensor.getDistance().getValueAsDouble());
    inputs.CANrangeDistanceInches = canRangeDistance.getValueAsDouble();
    // Logger.recordOutput(
    //     "Debug Elevator/Left Motor Stator Current",
    // leader.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Debug Elevator/Right Motor Stator Current",
    //     follower.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Debug Elevator/Left Motor Supply Current",
    // leader.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Debug Elevator/Right Motor Supply Current",
    //     follower.getSupplyCurrent().getValueAsDouble());

    inputs.positionInch =
        Conversions.motorRotToInches(
            elevatorPosition.getValueAsDouble(),
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);

    inputs.elevatorVelocityInchesPerSecond =
        Conversions.motorRotToInches(
            elevatorVelocity.getValueAsDouble() * 60.,
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrentAmps.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrentAmps.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();

    inputs.positionSetpointInch = positionSetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;

    leader.setControl(
        new PositionVoltage(
                Conversions.inchesToMotorRot(
                    position, 5.5, SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO))
            .withFeedForward(ffVolts));
  }

  @Override
  public void zeroElevator() {
    leader.setPosition(0);
  }

  @Override
  public void stop() {
    this.positionSetpoint = elevatorPosition.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePIDF(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    // config.GravityType = GravityTypeValue.Elevator_Static;
    // config.kG = kG;
    // config.kV = kV;

    leader.getConfigurator().apply(config);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (brake) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }
}
