package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

public class SetClawLevel extends ParallelCommandGroup {
<<<<<<< HEAD
  public SetClawLevel(ReefHeight scoringLevel, Elevator elevator, CoralScorerArm pivot) {
    addCommands(
        elevator.setElevatorTarget(scoringLevel.height, 0.5),
        pivot.setArmTarget(scoringLevel.pitch, 1));
=======
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final ElevatorState scoringLevel;

  public SetClawLevel(ElevatorState scoringLevel, Elevator elevator, CoralScorerArm pivot) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.scoringLevel = scoringLevel;
    addCommands(pivot.setArmTarget(50, 1));
>>>>>>> refactor-autons
  }
}
