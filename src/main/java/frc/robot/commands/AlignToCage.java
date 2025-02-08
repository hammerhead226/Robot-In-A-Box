package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AlignToCage extends Command {
  private Drive drive;
  private Command pathCommand;
  private List<Pose2d> possiblePoses =
      new ArrayList<>(Arrays.asList(Barge.closeCage, Barge.middleCage, Barge.farCage));

  public AlignToCage(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d targetPose = drive.getPose().nearest(possiblePoses);
    
    Logger.recordOutput("AlignToCageTargetPose: ", targetPose);
    //targetPose = new Pose2d(targetPose.getTranslation(), rotation2d);
    Rotation2d rotation = new Rotation2d(Math.PI);

    targetPose = new Pose2d(targetPose.getTranslation(), rotation);
    Translation2d buffer = new Translation2d(0.25, 0);
    targetPose = targetPose.transformBy(new Transform2d(buffer, new Rotation2d(0)));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), targetPose);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.5, targetPose.getRotation()));
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }
    @Override
  public boolean isFinished() {
    return false;
  }
}
