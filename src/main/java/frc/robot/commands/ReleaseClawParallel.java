package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

public class ReleaseClawParallel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final ScoralArm pivot;
  private final ScoralRollers flywheel;
  private final ReefHeight scoringLevel;

  public ReleaseClawParallel(
      ReefHeight scoringLevel, Elevator elevator, ScoralArm pivot, ScoralRollers flywheel) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.scoringLevel = scoringLevel;
    addCommands(
        elevator.setElevatorTarget(scoringLevel.height, 0.5),
        pivot.setArmTarget(scoringLevel.pitch, 1),
        flywheel.runVoltsCommmand(12));
  }
}
