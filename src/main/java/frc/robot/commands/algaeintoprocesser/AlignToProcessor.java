package frc.robot.commands.algaeintoprocesser;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class AlignToProcessor extends Command {
    Elevator elevator;
    Drive drive;
    Command pathCommand;
    public AlignToProcessor(Elevator elevator, Drive drive) {
        this.drive = drive;
        this.elevator = elevator;
        addRequirements(elevator, drive);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = FieldConstants.Processor.centerFace;
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
