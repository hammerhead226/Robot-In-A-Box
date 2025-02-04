// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.flywheel.Flywheel;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFromSource extends Command {
  private final Flywheel coralIntake;
  /** Creates a new IntakeFromSource. */
  public IntakeFromSource(Flywheel coralIntake) {
    this.coralIntake = coralIntake;
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (coralIntake.seesCoral() == CoralState.SENSOR || coralIntake.seesCoral() == CoralState.CURRENT) {
      coralIntake.stopRollers();
    } else {
      coralIntake.runRollers(12);
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralState.seesNote() == CoralState.SENSOR || CoralState.seesNote() == CoralState.CURRENT;
  }
}
