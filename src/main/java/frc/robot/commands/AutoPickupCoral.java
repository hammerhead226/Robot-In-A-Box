import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.arms.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;

//UNFINISHED
/*
public class AutoPickupCoral extends Command {
    private final Drive drive;
    private final LED led;
    private final Vision vision;
    private final CoralIntake coralIntake;

    Command pathCommand;

    public AutoPickupCoral(CoralIntake coralIntake, Vision vision, Drive drive, LED led) {
        this.coralIntake = coralIntake;
        this.vision = vision;
        this.drive = drive;
        this.led = led;

        drive.getPose().getTranslation();
    }

    @Override
    public void initialize() {
        
        led.setState(SubsystemConstants.LED_STATE.ALIGNING);

        coralIntake.runFlywheel();

        Translation2d targetTranslation2d = vision.getCoralFieldPosition();
        Rotation2d targetRotation2d = new Rotation2d(
            targetTranslation2d.getX()-drive.getPose().getX(),
            targetTranslation2d.getY()-drive.getPose().getY()
            );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            drive.getPose(),
            new Pose2d(targetTranslation2d, targetRotation2d)
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.5, targetRotation2d)
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
        //detect if object has been grabbed
    }
}
*/
