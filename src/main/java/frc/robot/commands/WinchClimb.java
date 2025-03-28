// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.Winch;
import frc.robot.util.SlewRateLimiter;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WinchClimb extends Command {
  /** Creates a new WinchClimb. */
  Winch winch;

  ClimberArm climberArm;
  BooleanSupplier continueWinching;
  SlewRateLimiter voltageSlewRateLimiter;
  double timeTest;

  public WinchClimb(Winch winch, ClimberArm climberArm, BooleanSupplier continueWinching) {
    this.climberArm = climberArm;
    this.winch = winch;
    this.continueWinching = continueWinching;

    addRequirements(winch, climberArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberArm.isWinching = true;
    voltageSlewRateLimiter = new SlewRateLimiter(0.2);
    timeTest = Timer.getFPGATimestamp();
    climberArm.setArmGoal(140);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (winch.getStatorCurrentAmps() < 25 || climberArm.getArmPositionDegs() < 110) {
      // if (Math.abs(timeTest - Timer.getFPGATimestamp()) < 3) {
      // winch.runVolts(-6);
    // } else {
      winch.runVolts(getWinchVoltage(climberArm.getArmPositionDegs()));
    // }
  }

  public double getWinchVoltage(double climberArmDegs) {
    return MathUtil.clamp(14.809-0.4535*climberArmDegs+0.00259*Math.pow(climberArmDegs, 2), -6, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winch.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !continueWinching.getAsBoolean();
  }
}
