// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj.Timer;
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
  Drive drive;

  ScoralArm scoralArm;

  boolean isRight;
  Pose2d atPose;
  double reefSensorDistance;
  double branchSensorDistance;
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

  Timer timer;
  double branchSensorForwardEffort = 0;
  boolean keepPID = true;
  double pidEndTime;

  public AdjustToReefPost(
      Drive drive, ScoralArm scoralArm, boolean isRight, BooleanSupplier triggerPressed) {
    this.drive = drive;
    this.scoralArm = scoralArm;
    this.isRight = isRight;
    this.triggerPressed = triggerPressed;
    this.timer = new Timer();
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    drive.isAutoAlignDone = false;

    sensorForwardPID.setTolerance(0.5);
    odometryForwardPID.setTolerance(Units.inchesToMeters(0.5));
    odometrySidePID.setTolerance(Units.inchesToMeters(0.5));
    rotationPID.setTolerance(5);

    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();
    atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET, -0.1),
            Rotation2d.kZero);

    odometryForwardPID.reset(drive.getPose().getX());
    odometrySidePID.reset(drive.getPose().getY());
    rotationPID.reset(drive.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("ajust to post", atPose);
    distanceToGoal = drive.getPose().getTranslation().getDistance(atPose.getTranslation());
    reefSensorDistance = drive.getSensorDistanceInches();
    branchSensorDistance = scoralArm.getCANRangeDistance();
    angleToGoal = drive.getRotation().getDegrees() - atPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees();

    if (!drive.isSlowMode()) {
      forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      rotationSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
    } else {
      forwardSlewRateLimiter.changeRateLimit(2);
      sidewaysSlewRateLimiter.changeRateLimit(2);
      rotationSlewRateLimiter.changeRateLimit(14);
    }

    shouldAlign = drive.shouldEndPath();

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double odometryForwardEffort;
    double odometrySideEffort;
    if (drive.getPose().getTranslation().getDistance(atPose.getTranslation())
            >= Units.inchesToMeters(1)
        && keepPID) {
      odometryForwardEffort = odometryForwardPID.calculate(drive.getPose().getX(), atPose.getX());
      odometrySideEffort = odometrySidePID.calculate(drive.getPose().getY(), atPose.getY());
    } else {
      pidEndTime = Timer.getFPGATimestamp();
      odometryForwardEffort = 0;
      odometrySideEffort = 0;
    }

    double reefSensorSideEffort = 0;
    double rotationEffort =
        Units.degreesToRadians(
            rotationPID.calculate(
                drive.getPose().getRotation().getDegrees(), atPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees()));
    if (Math.abs(pidEndTime - Timer.getFPGATimestamp()) > 1.5) {
      branchSensorForwardEffort = 0;
    } else {
      branchSensorForwardEffort = -0.31;
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
    if (reefSensorDistance <= 14 && (branchSensorDistance >= 13 && branchSensorDistance <= 18)) {

      drive.isAutoAlignDone = true;
    }
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !triggerPressed.getAsBoolean()
        || (reefSensorDistance <= 14 && (branchSensorDistance >= 13 && branchSensorDistance <= 18) && Math.abs(angleToGoal) <= 5);
  }
}
