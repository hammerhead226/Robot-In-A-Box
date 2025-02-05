package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

public class SetClawLevel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final ReefHeight scoringLevel;

  public SetClawLevel(ReefHeight scoringLevel, Elevator elevator, CoralScorerArm pivot) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.scoringLevel = scoringLevel;
    addCommands(
        elevator.setElevatorTarget(scoringLevel.height, 0.5),
        pivot.setArmTarget(scoringLevel.pitch, 1));
  }
}
