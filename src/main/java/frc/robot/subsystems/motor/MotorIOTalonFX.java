package frc.robot.subsystems.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.commoniolayers.MotorIO;

public class MotorIOTalonFX implements MotorIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX leader;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderStatorCurrentAmps;
  private final StatusSignal<Current> leaderSupplyCurrentAmps;

  public MotorIOTalonFX(int id) {

    leader = new TalonFX(id);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leader.getConfigurator().apply(config);

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderStatorCurrentAmps = leader.getStatorCurrent();
    leaderSupplyCurrentAmps = leader.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrentAmps,
        leaderSupplyCurrentAmps);
    leader.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrentAmps,
        leaderSupplyCurrentAmps);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrentAmps.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();

    // Logger.recordOutput(
    //     "Debug Scoral Rollers/Motor Stator Current",
    // leader.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Debug Scoral Rollers/Motor Supply Current",
    // leader.getSupplyCurrent().getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(-volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  ____________
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
  }
}
