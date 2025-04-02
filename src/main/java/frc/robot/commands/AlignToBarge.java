// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
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

  double distanceToTarget;

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
    drive.isBargeAutoAlignDone = false;
    isPathFinished = false;
    led.setState(LED_STATE.FLASHING_RED);

    targetPose =
        DriverStation.getAlliance().get() == Alliance.Red
            ? new Pose2d(
                10, MathUtil.clamp(drive.getPose().getY(), 0.5, 2.87), Rotation2d.fromDegrees(90))
            : new Pose2d(
                7.6, MathUtil.clamp(drive.getPose().getY(), 5, 7.5), Rotation2d.fromDegrees(90));

    PathConstraints pathConstraints;

    pathConstraints = new PathConstraints(2.5, 2.25, Math.toRadians(180), Math.toRadians(200));

    Logger.recordOutput("Debug OTF Paths/Barge Align", targetPose);

    pointsTooClose =
        drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) <= 0.01;

    if (!pointsTooClose) {
      pathCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints);

      pathCommand.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToTarget = drive.getPose().getTranslation().getDistance(targetPose.getTranslation());
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
      if (distanceToTarget <= Units.inchesToMeters(4)) {
        drive.isBargeAutoAlignDone = true;
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !continuePath.getAsBoolean() || pointsTooClose || isPathFinished;
  }
}
