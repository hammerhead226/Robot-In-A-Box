
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlignToReefAuto extends Command {
    private final Drive drive;
    private final LED led;
    
    Command pathCommand;

    public AlignToReefAuto(Drive drive, LED led) {
        this.drive = drive;
        this.led = led;
    
    
        addRequirements(drive, led);
    }

    @Override
    public void initialize() {
        // led.setState(SubsystemConstants.LED_STATE.ALIGNING);
        Pose2d targetPose = getNearestReefSide();
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

    private Pose2d getNearestReefSide() {
        Translation2d start = getAllianceReefPosition();
        Translation2d end = drive.getPose().getTranslation(); 
        Translation2d v = start.minus(end);
        Rotation2d angle = new Rotation2d(v.getX(),v.getY());

        // might look something like
        // return new ReefSide(Math.floor(getRevolutions * 6));
    }

    @Override
    public void execute() {
        //we could switch to PID as we get closer to the wall? for instance 
        //if both CANRanges measure a short distance
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        // or any other way we can measure "close enough" to desired position
        return false;
}