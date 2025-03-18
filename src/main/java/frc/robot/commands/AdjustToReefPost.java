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
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.util.SlewRateLimiter;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustToReefPost extends Command {
  /** Creates a new AdjustToReefPost. */
  enum AlignState {
    BRANCH_SENSOR,
    ODOMETRY
  }

  Drive drive;
  ScoralArm scoralArm;

  boolean isRight;
  Pose2d offsetPose;
  Pose2d odometryTargetPose;

  AlignState state;

  double reefSensorDistance;
  double branchSensorDistance;
  double angleToGoal;

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

  BooleanSupplier triggerPressed;
  ChassisSpeeds chassisSpeeds;

  boolean keepPID = true;
  Pose2d pidEndPose = new Pose2d();

  boolean isAligned;

  public AdjustToReefPost(
      Drive drive, ScoralArm scoralArm, boolean isRight, BooleanSupplier triggerPressed) {
    this.drive = drive;
    this.scoralArm = scoralArm;
    this.isRight = isRight;
    this.triggerPressed = triggerPressed;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isAutoAlignDone = false;

    state = scoralArm.isCANRangeConnected() ? AlignState.BRANCH_SENSOR : AlignState.ODOMETRY;
    // state = AlignState.BRANCH_SENSOR;

    isAligned = false;

    sensorForwardPID.setTolerance(0.5);
    odometryForwardPID.setTolerance(Units.inchesToMeters(0.5));
    odometrySidePID.setTolerance(Units.inchesToMeters(0.5));
    rotationPID.setTolerance(5);

    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();

    offsetPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET, -0.1),
            Rotation2d.kZero);

    odometryTargetPose =
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
    Logger.recordOutput("Aligning State", state);

    reefSensorDistance = drive.getCANRangeDistanceInches();
    branchSensorDistance = scoralArm.getCANRangeDistance();
    angleToGoal =
        drive.getRotation().getDegrees()
            - offsetPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees();

    if (!drive.isSlowMode()) {
      forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      rotationSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
    } else {
      forwardSlewRateLimiter.changeRateLimit(2);
      sidewaysSlewRateLimiter.changeRateLimit(2);
      rotationSlewRateLimiter.changeRateLimit(14);
    }

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double odometryForwardEffort = 0;
    double odometrySideEffort = 0;
    double branchSensorForwardEffort = 0;
    double reefSensorSideEffort =
        drive.isCANRangeConnected()
            ? -MathUtil.clamp(
                sensorForwardPID.calculate(drive.getCANRangeDistanceInches(), 12.5), -1, 1)
            : 0;
    // double reefSensorSideEffort = 0;
    double rotationEffort =
        Units.degreesToRadians(
            rotationPID.calculate(
                drive.getPose().getRotation().getDegrees(),
                offsetPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees()));

    if (state == AlignState.ODOMETRY) {
      isAligned =
          drive.getPose().getTranslation().getDistance(odometryTargetPose.getTranslation())
                  <= Units.inchesToMeters(0.5)
              && Math.abs(angleToGoal) <= 5;
      odometryForwardEffort =
          odometryForwardPID.calculate(drive.getPose().getX(), odometryTargetPose.getX());
      odometrySideEffort =
          odometrySidePID.calculate(drive.getPose().getY(), odometryTargetPose.getY());
      branchSensorForwardEffort = 0;
    } else {
      isAligned =
          (reefSensorDistance <= 14
              && (branchSensorDistance >= 13 && branchSensorDistance <= 18)
              && Math.abs(angleToGoal) <= 5);
      if (drive.getPose().getTranslation().getDistance(offsetPose.getTranslation())
              >= Units.inchesToMeters(1)
          && keepPID) {
        odometryForwardEffort =
            odometryForwardPID.calculate(drive.getPose().getX(), offsetPose.getX());
        odometrySideEffort = odometrySidePID.calculate(drive.getPose().getY(), offsetPose.getY());
      } else if (drive.getPose().getTranslation().getDistance(offsetPose.getTranslation())
          < Units.inchesToMeters(1)) {
        pidEndPose = drive.getPose();
        keepPID = false;
        odometryForwardEffort = 0;
        odometrySideEffort = 0;
      }
      if (!keepPID
          && Math.abs(pidEndPose.getTranslation().getDistance(drive.getPose().getTranslation()))
              > Units.inchesToMeters(4)) {
        odometryForwardEffort = 0;
        odometrySideEffort = 0;
        branchSensorForwardEffort = 0;
        state = AlignState.ODOMETRY;
      } else if (!keepPID
      && Math.abs(pidEndPose.getTranslation().getDistance(drive.getPose().getTranslation()))
          <= Units.inchesToMeters(4)) {
        odometryForwardEffort = 0;
        odometrySideEffort = 0;
        branchSensorForwardEffort = -0.31;
      }
    }

    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            odometryForwardEffort,
            odometrySideEffort,
            rotationEffort,
            isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation());

    drive.runVelocity(
        new ChassisSpeeds(
                forwardSlewRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
                sidewaysSlewRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond),
                chassisSpeeds.omegaRadiansPerSecond)
            .plus(new ChassisSpeeds(branchSensorForwardEffort, reefSensorSideEffort, 0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isAligned) {
      drive.isAutoAlignDone = true;
    }
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !triggerPressed.getAsBoolean() || isAligned;
  }
}
