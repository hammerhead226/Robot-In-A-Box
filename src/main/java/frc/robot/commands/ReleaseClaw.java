package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.elevator.Elevator;

public class ReleaseClaw extends Command {
  private final Elevator elevator;
  private final ReefHeight scoringLevel;

  public ReleaseClaw(ReefHeight scoringLevel, Elevator elevator) {
    this.elevator = elevator;
    this.scoringLevel = scoringLevel;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.setElevatorTarget(scoringLevel.height, 1);
  }
}
