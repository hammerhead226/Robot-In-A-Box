// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoral.ScoralRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveDriveTrainSide extends Command {
  /** Creates a new MoveDriveTrainSide. */
  Drive drive;

  ScoralRollers scoralRollers;
  Timer timer;
  boolean isRight;

  public MoveDriveTrainSide(Drive drive, ScoralRollers scoralRollers, boolean isRight) {
    this.drive = drive;
    this.scoralRollers = scoralRollers;
    this.isRight = isRight;
    timer = new Timer();
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    drive.runVelocity(new ChassisSpeeds(isRight ? 2 : -2, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.20) || scoralRollers.seesCoral() == CoralState.SENSOR;
  }
}
