// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachReefPerpendicular extends Command {
  private final Drive drive;
  Command pathCommand;
  /** Creates a new ApproachReefPerpendicular. */
  public ApproachReefPerpendicular(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose =
        DriveCommands.rotateAndNudge(
            drive.getLastReefFieldPose(), new Translation2d(-0.5, 0), new Rotation2d(Math.PI));
    // new Pose2d(
    //     drive.getNearestSide().getTranslation().minus(drive.getOffset()),
    //     drive.getNearestSide().getRotation());
    // new Pose2d(nearestSide.getTranslation().minus(offset), nearestSide.getRotation())
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), targetPose);
    List<EventMarker> eventMarkers = new ArrayList<>();
    eventMarkers.add(
        new EventMarker(
            "test trigger",
            0.7,
            new InstantCommand(() -> System.out.println("event marker hit!"))));
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new ArrayList<>(),
            new ArrayList<>(),
            new ArrayList<>(),
            eventMarkers,
            new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.5, targetPose.getRotation()),
            false);
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
