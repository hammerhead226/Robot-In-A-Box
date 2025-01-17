package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.constants.SubsystemConstants;

public class AutoAlignToSource extends Command {
  private final Drive drive;
  private final LED led;

  Command pathCommand;

  public AutoAlignToSource(Drive drive, LED led) {
    this.drive = drive;
    this.led = led;

    drive.getPose().getTranslation();
  }

  @Override
  public void initialize() {
    //led.setState(SubsystemConstants.LED_STATE.ALIGNING);

    // Translation2d targetTranslation2d = drive.getPose() > FieldConstants.HEIGHT / 2.0 ? FieldConstants.Sources.Lower.Position : FieldConstants.Sources.Higher.Position;
    // Rotation2d targetRotation2d = new Rotation2d(
    //   targetTranslation2d.getX()-drive.getPose().getX(),
    //   targetTranslation2d.getY()-drive.getPose().getY()
    //   );
    
    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //   drive.getPose(),
    //   new Pose2d(targetTranslation2d, targetRotation2d)
    // );

    // TODO rewrite this with the correct names onces FieldConstants is done
    Pose2d targetPose = drive.getPose() > FieldConstants.HEIGHT / 2.0 ? FieldConstants.Sources.Lower.Position : FieldConstants.Sources.Higher.Position;
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      drive.getPose(), 
      targetPose
    );


    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.5, targetPose.getRotation())
    );
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
    pathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
