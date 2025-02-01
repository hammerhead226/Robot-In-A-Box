package frc.robot.commands.algaeintosource;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;

public class ReleaseAlgae extends Command {
  CoralScorerFlywheel coralScorerFlywheel;

  double FLYWHEEL_RELEASE_VOLTS = 5;

  public ReleaseAlgae(CoralScorerFlywheel coralScorerFlywheel) {
    this.coralScorerFlywheel = coralScorerFlywheel;

    addRequirements(coralScorerFlywheel);
  }

  @Override
  public void initialize() {
    coralScorerFlywheel.runVolts(
        SubsystemConstants.CoralScorerConstants.CoralScorerFlywheelConstants.DEFAULT_VOLTS);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this detect the algae has been released via volt threshold
    return false;
  }
}
