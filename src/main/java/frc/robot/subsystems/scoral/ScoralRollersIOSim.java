package frc.robot.subsystems.scoral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.FlywheelIO;

public class ScoralRollersIOSim implements FlywheelIO {
  // CHANGE THESE VALUES TO MATCH YOUR MOTOR AND GEARBOX
  private int gearBoxMotorCount = 1;
  private double gearing = 1.5;
  //  private double momentOfInertia = 1;
  private DCMotor motor = DCMotor.getKrakenX60Foc(gearBoxMotorCount);
  //  private double[] stds = {1, 2, 3};

  private DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, gearing), motor);

  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts,
              clampedValueLowVolts,
              clampedValueHighVolts);
      sim.setInputVoltage(appliedVolts);
    }
    // sim.setInputVoltage(appliedVolts);
    sim.update(SubsystemConstants.LOOP_PERIOD_SECONDS);

    inputs.positionRad = 0.0;
    // inputs.velocityRadPerSec =
    // Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    inputs.velocityRadPerSec = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
