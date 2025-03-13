// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.scoral.ScoralArm;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachReef extends Command {
  private final Drive drive;
  private final Elevator elevator;
  private final ScoralArm scoralArm;
  private final SuperStructure superStructure;
  private final boolean isRight;
  Command pathCommand;
  BooleanSupplier continuePath;

  Pose2d atPose;
  Pose2d awayPose;

  boolean shouldPID = false;
  /** Creates a new ApproachReef. */
  public ApproachReef(
      Drive drive,
      Elevator elevator,
      ScoralArm scoralArm,
      LED led,
      SuperStructure superStructure,
      boolean isRight,
      BooleanSupplier continuePath) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    this.superStructure = superStructure;
    this.isRight = isRight;
    this.continuePath = continuePath;
    addRequirements(drive, elevator, scoralArm, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();

    superStructure.setWantedState(superStructure.getLastReefState());

    awayPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AWAY_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);
    // offset the path's setpoint a bit to allow pid to do the rest of work while avoiding bug of
    // both current pose and atpose being the same which causes reboot
    // atPose =
    //     DriveCommands.rotateAndNudge(
    //         reefPose,
    //         new Translation2d(
    //             SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET - 0.1,
    //             SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
    //         Rotation2d.kZero);
    atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);

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
      Translation2d v = awayPose.getTranslation().minus(drive.getPose().getTranslation());
      currentPoseFacingVelocity =
          new Pose2d(drive.getPose().getTranslation(), new Rotation2d(v.getX(), v.getY()));
    }

    List<Waypoint> waypoints;
    List<RotationTarget> holomorphicRotations;
    List<EventMarker> eventMarkers = new ArrayList<>();
    PathConstraints pathConstraints;

    if (atPose.getRotation().minus(drive.getRotation()).getDegrees() <= 45) {
      pathConstraints = new PathConstraints(2.5, 2, 150, 226);
    } else {
      pathConstraints = new PathConstraints(1.5, 1, 100, 180);
    }

    if (!drive.isNearReef()) {
      waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, awayPose, atPose);
      holomorphicRotations =
          Arrays.asList(
              new RotationTarget(1.0, awayPose.getRotation().plus(Rotation2d.kCW_90deg)),
              new RotationTarget(1.7, atPose.getRotation().plus(Rotation2d.kCW_90deg)));
    } else {
      waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, atPose);
      holomorphicRotations =
          Arrays.asList(new RotationTarget(0.7, atPose.getRotation().plus(Rotation2d.kCW_90deg)));
    }
    Logger.recordOutput("Debug OTF Paths/Reef Align", atPose);

    eventMarkers.add(
        new EventMarker("score coral command", 0.6, superStructure.getSuperStructureCommand()));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            holomorphicRotations,
            new ArrayList<>(),
            new ArrayList<>(),
            eventMarkers,
            pathConstraints, // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0, atPose.getRotation().rotateBy(Rotation2d.fromDegrees(-90))),
            false);
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    shouldPID = drive.shouldPIDAlign();
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shouldPID = drive.shouldPIDAlign();
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
    if (drive.getPose().getTranslation().getDistance(atPose.getTranslation()) <= 0.2) {
      superStructure.nextState();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished() || !continuePath.getAsBoolean();
  }
}
