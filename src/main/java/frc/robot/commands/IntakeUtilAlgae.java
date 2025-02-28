// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.subsystems.scoral.ScoralRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeUtilAlgae extends Command {
  /** Creates a new IntakeUtilAlgae. */
  ScoralRollers scoralRollers;

  public IntakeUtilAlgae(ScoralRollers scoralRollers) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoralRollers = scoralRollers;
    addRequirements(scoralRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoralRollers.runVolts(-2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scoralRollers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scoralRollers.seesAlgae() == AlgaeState.CURRENT;
  }
}
