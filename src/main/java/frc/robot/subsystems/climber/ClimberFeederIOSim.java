package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SubsystemConstants;

public class ClimberFeederIOSim implements ClimberFeederIO {
  private final DCMotor motor = DCMotor.getKrakenX60(1);
  // Corrected measurementStdDevs array with 2 elements
  private DCMotorSim sim =
<<<<<<< Updated upstream
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motor, 1, 1), motor, new double[] {0.0, 0.0});
=======
<<<<<<< HEAD
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, 1), motor, 0.0, 0.0);
=======
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motor, 1, 1), motor, new double[] {0.0, 0.0});
>>>>>>> 683f32a72d23f1ebcddf549f9fd5acb41a72a1c8
>>>>>>> Stashed changes

  private PIDController pid = new PIDController(0, 0, 0);

  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double velocitySetpointRPS = 0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM() / 60) + ffVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);

    sim.update(SubsystemConstants.LOOP_PERIOD_SECONDS);

    inputs.velocitySetpointRPM = velocitySetpointRPS * 60.;
    inputs.feederVelocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.ffVolts = ffVolts;
    this.velocitySetpointRPS = velocityRPS;
    pid.setSetpoint(velocityRPS);
  }

  @Override
  public void stop() {
    setVelocityRPS(0, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
