package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

public class SetClawLevel extends ParallelCommandGroup {
  public SetClawLevel(ReefHeight scoringLevel, Elevator elevator, CoralScorerArm pivot) {
    addCommands(
        elevator.setElevatorTarget(scoringLevel.height, 0.5),
        pivot.setArmTarget(scoringLevel.pitch, 1));
  }
}
