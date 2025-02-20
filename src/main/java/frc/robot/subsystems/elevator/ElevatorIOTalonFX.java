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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private double positionSetpoint;
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final CANrange extenderDistance;

  public ElevatorIOTalonFX(int lead, int follow) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leader = new TalonFX(lead, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(follow, SubsystemConstants.CANIVORE_ID_STRING);
    leader.getConfigurator().apply(config);

    positionSetpoint = SubsystemConstants.ElevatorConstants.RETRACT_SETPOINT_INCH;

    follower.setControl(new Follower(lead, true));

    elevatorPosition = leader.getPosition();
    elevatorVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
    if (inputs.elevatorPositionInch <= 34) {
      inputs.elevatorPositionInch =
          Units.metersToInches(extenderDistance.getDistance().getValueAsDouble());
    }
    inputs.elevatorPositionInch =
        Conversions.motorRotToInches(
            elevatorPosition.getValueAsDouble(),
            5.97,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    inputs.elevatorVelocityInchesPerSecond =
        Conversions.motorRotToInches(
            elevatorVelocity.getValueAsDouble() * 60.,
            5.97,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionSetpointInch = positionSetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    leader.setVoltage(volts);
  }
  // weird how we give the setpoint in meters and it sets it to meters in sim?
  // we'll have to see what the exact bug is but for now work with meters
  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;
    leader.setControl(
        new PositionVoltage(
            Conversions.MetersToMotorRot(
                position, 5.97, SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO)));
  }

  @Override
  public void stop() {
    this.positionSetpoint = elevatorPosition.getValueAsDouble();
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

  @Override
  public void setBrakeMode(boolean brake) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (brake) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
  }
}
