// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBarge extends Command {
  private final Drive drive;
  private final LED led;
  private boolean pointsTooClose;

  Command pathCommand;
  BooleanSupplier continuePath;

  Pose2d targetPose;

  boolean isPathFinished;
  boolean skipPath;
  /** Creates a new ApproachReef. */
  public AlignToBarge(
      Drive drive, LED led, SuperStructure superStructure, BooleanSupplier continuePath) {
    this.drive = drive;
    this.led = led;
    this.continuePath = continuePath;
    addRequirements(drive, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPathFinished = false;
    skipPath = false;
    // led.setState(LED_STATE.FLASHING_RED);

    targetPose =
        DriverStation.getAlliance().get() == Alliance.Red
            ? new Pose2d(10, MathUtil.clamp(drive.getPose().getY(), 0.5, 2.87), Rotation2d.kZero)
            : new Pose2d(7.6, MathUtil.clamp(drive.getPose().getY(), 5, 7.5), Rotation2d.kZero);

    ChassisSpeeds fieldRelChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    double chassisSpeedSingular =
        Math.hypot(
            fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);

    Pose2d currentPoseFacingVelocity;
    if (chassisSpeedSingular >= 0.2) {
      currentPoseFacingVelocity =
          new Pose2d(
              drive.getPose().getTranslation(),
              new Rotation2d(
                  fieldRelChassisSpeeds.vxMetersPerSecond,
                  fieldRelChassisSpeeds.vyMetersPerSecond));
    } else {
      Translation2d v = targetPose.getTranslation().minus(drive.getPose().getTranslation());
      currentPoseFacingVelocity =
          new Pose2d(drive.getPose().getTranslation(), new Rotation2d(v.getX(), v.getY()));
    }

    List<Waypoint> waypoints;
    List<RotationTarget> holomorphicRotations;
    List<EventMarker> eventMarkers = new ArrayList<>();
    PathConstraints pathConstraints;

    List<ConstraintsZone> constraintsZones = new ArrayList<>();
    pathConstraints = new PathConstraints(2.5, 2.5, 180, 200);
    waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, targetPose);
    holomorphicRotations = Arrays.asList(new RotationTarget(0.7, Rotation2d.fromDegrees(90)));

    Logger.recordOutput("Debug OTF Paths/Reef Align", targetPose);

    pointsTooClose =
        drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) <= 0.01;

    if (!pointsTooClose) {
      PathPlannerPath path =
          new PathPlannerPath(
              waypoints,
              holomorphicRotations,
              new ArrayList<>(),
              constraintsZones,
              eventMarkers,
              pathConstraints, // these numbers from last year's code
              null, // The ideal starting state, this is only relevant for pre-planned paths, so
              // can
              // be null for on-the-fly paths.
              new GoalEndState(0, Rotation2d.fromDegrees(90)),
              false);
      path.preventFlipping = true;

      pathCommand = AutoBuilder.followPath(path);

      pathCommand.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!pointsTooClose) {
      isPathFinished = pathCommand.isFinished();
      pathCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (!pointsTooClose) {
      pathCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !continuePath.getAsBoolean() || pointsTooClose || isPathFinished || skipPath;
  }
}
