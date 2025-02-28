// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringProccessorSequential extends SequentialCommandGroup {
  private final ScoralArm csArm;
  private final Elevator elevator;

  public ScoringProccessorSequential(ScoralArm csArm, Elevator elevator) {
    this.elevator = elevator;
    this.csArm = csArm;

    addCommands(elevator.setElevatorTarget(4, 2), csArm.setArmTarget(20, 2));
  }
}
