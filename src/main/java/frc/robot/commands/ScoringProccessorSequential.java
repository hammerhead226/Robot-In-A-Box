// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringProccessorSequential extends SequentialCommandGroup {
  private final CoralScorerFlywheel csFlywheel;
  private final CoralScorerArm csArm;
  private final Elevator elevator; 

  public ScoringProccessorSequential(
    CoralScorerFlywheel csFlywheel, CoralScorerArm csArm, Elevator elevator) {
    this.csFlywheel = csFlywheel;
    this.elevator = elevator;
    this.csArm = csArm;

    addCommands(
    elevator.setElevatorTarget(0.25, 0.01),
    csArm.setArmTarget(190, 2),
    new InstantCommand(() -> csFlywheel.runVolts(5)));
  }
}
