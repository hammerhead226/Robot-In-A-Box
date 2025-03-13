// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustToReefPost extends Command {
  /** Creates a new AdjustToReefPost. */
  Drive drive;

  boolean isRight;
  Pose2d atPose;

  ProfiledPIDController forwardPID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  ProfiledPIDController sidePID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));

       SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(0.8);
  SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(0.8);


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
    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();
    atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);

    forwardPID.reset(drive.getPose().getX());
    sidePID.reset(drive.getPose().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!drive.isSlowMode()) {
      forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
    } else {
      forwardSlewRateLimiter.changeRateLimit(2);
      sidewaysSlewRateLimiter.changeRateLimit(2);
    }
    shouldAlign = drive.shouldPIDAlign();

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardPID.calculate(drive.getPose().getX(), atPose.getX()),
            sidePID.calculate(drive.getPose().getY(), atPose.getY()),
            0,
            isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation());

    drive.runVelocity(
        new ChassisSpeeds(forwardSlewRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond), sidewaysSlewRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !triggerPressed.getAsBoolean() || !shouldAlign;
  }
}
