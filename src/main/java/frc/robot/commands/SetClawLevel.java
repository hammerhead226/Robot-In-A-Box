package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

public class SetClawLevel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final ElevatorState scoringLevel;

  public SetClawLevel(ElevatorState scoringLevel, Elevator elevator, CoralScorerArm pivot) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.scoringLevel = scoringLevel;
    addCommands(pivot.setArmTarget(50, 1));
  }
}
