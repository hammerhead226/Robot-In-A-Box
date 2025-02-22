package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;

public class SetClawLevel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final ScoralArm pivot;
  private final ElevatorState scoringLevel;

  public SetClawLevel(ElevatorState scoringLevel, Elevator elevator, ScoralArm pivot) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.scoringLevel = scoringLevel;
    addCommands(pivot.setArmTarget(50, 1));
  }
}
