package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;

public class WinchIOTalonFX implements WinchIO {
  private static final double GEAR_RATIO = 9;

  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;

  public WinchIOTalonFX(int leadID, int followID) {

    leader = new TalonFX(leadID, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(followID, SubsystemConstants.CANIVORE_ID_STRING);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leadID, true));

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    leaderCurrent = leader.getSupplyCurrent();
    followerCurrent = follower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.winchPositionRads =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.winchVelocityRPM = leaderVelocity.getValueAsDouble() / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps = leaderCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocityRPM(double velocityRadPerSec, double ffVolts) {
    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
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
