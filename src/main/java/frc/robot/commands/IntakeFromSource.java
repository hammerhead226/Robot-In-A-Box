// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFromSource extends Command {
  /** Creates a new IntakeFromSource. */
  private final CoralScorerFlywheel coralIntake;
  private final Elevator elevator;

  public IntakeFromSource(CoralScorerFlywheel coralIntake, Elevator elevator) {
    this.coralIntake = coralIntake;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (coralIntake.seesCoral() == CoralState.SENSOR
        || coralIntake.seesCoral() == CoralState.CURRENT) {
      coralIntake.flywheelStop();
    } else {
      coralIntake.runVolts(12);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.flywheelStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralIntake.seesCoral() == CoralState.SENSOR
        || coralIntake.seesCoral() == CoralState.CURRENT;
  }
}
