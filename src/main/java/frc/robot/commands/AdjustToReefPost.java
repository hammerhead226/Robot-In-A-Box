// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SlewRateLimiter;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustToReefPost extends Command {
  /** Creates a new AdjustToReefPost. */
  Drive drive;

  boolean isRight;
  Pose2d atPose;
  double sensorDistance;
  double angleToGoal;
  double distanceToGoal;
  PIDController sensorForwardPID = new PIDController(0.1, 0, 0);
  ProfiledPIDController odometryForwardPID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  ProfiledPIDController odometrySidePID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  ProfiledPIDController rotationPID =
      new ProfiledPIDController(2.9, 0., 0.2, new TrapezoidProfile.Constraints(200, 300));

  SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(0.8);
  SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(0.8);
  SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(0.8);

  boolean shouldAlign;
  BooleanSupplier triggerPressed;
  ChassisSpeeds chassisSpeeds;

  public AdjustToReefPost(Drive drive, boolean isRight, BooleanSupplier triggerPressed) {
    this.drive = drive;
    this.isRight = isRight;
    this.triggerPressed = triggerPressed;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isAutoAlignDone = false;

    sensorForwardPID.setTolerance(0.5);
    odometryForwardPID.setTolerance(Units.inchesToMeters(0.5));
    odometrySidePID.setTolerance(Units.inchesToMeters(0.5));
    rotationPID.setTolerance(5);

    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();
    atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);

    odometryForwardPID.reset(drive.getPose().getX());
    odometrySidePID.reset(drive.getPose().getY());
    rotationPID.reset(drive.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToGoal = drive.getPose().getTranslation().getDistance(atPose.getTranslation());
    sensorDistance = drive.getSensorDistanceInches();
    angleToGoal = drive.getRotation().getDegrees() - atPose.getRotation().getDegrees();

    if (!drive.isSlowMode()) {
      forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      rotationSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
    } else {
      forwardSlewRateLimiter.changeRateLimit(2);
      sidewaysSlewRateLimiter.changeRateLimit(2);
      rotationSlewRateLimiter.changeRateLimit(14);
    }

    shouldAlign = drive.shouldPIDAlign();

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            odometryForwardPID.calculate(drive.getPose().getX(), atPose.getX()),
            odometrySidePID.calculate(drive.getPose().getY(), atPose.getY())
                + (-MathUtil.clamp(
                    sensorForwardPID.calculate(drive.getSensorDistanceInches(), 13.23), -1, 1)),
            Units.degreesToRadians(
                rotationPID.calculate(
                    drive.getPose().getRotation().getDegrees(), atPose.getRotation().getDegrees())),
            isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation());

    drive.runVelocity(
        new ChassisSpeeds(
            forwardSlewRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
            sidewaysSlewRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond),
            chassisSpeeds.omegaRadiansPerSecond));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (distanceToGoal <= Units.inchesToMeters(0.5)) {

      drive.isAutoAlignDone = true;
    }
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !triggerPressed.getAsBoolean()
        || !shouldAlign
        || (sensorDistance <= 8 && distanceToGoal <= Units.inchesToMeters(0.5) && angleToGoal <= 5);
    // || !shouldAlign
    // || (distanceToGoal <= Units.inchesToMeters(0.5));
    //  && angleToGoal <= 5);
  }
}
