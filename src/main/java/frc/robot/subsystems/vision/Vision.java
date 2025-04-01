// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  private final Alert[] disconnectedAlerts;
  private static final double POSE_BUFFER_SIZE_SECONDS = 1.5;

  private final PowerDistribution PDH;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    SmartDashboard.putBoolean("Reset", false);
    SmartDashboard.putBoolean("Enable", false);

    PDH = new PowerDistribution(1, ModuleType.kRev);
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Command resetLimelight() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> PDH.setSwitchableChannel(false)),
        new WaitCommand(2),
        new InstantCommand(() -> PDH.setSwitchableChannel(true)),
        new InstantCommand(() -> SmartDashboard.putBoolean("Reset", false)));
  }

  public Command activateLimelight() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> PDH.setSwitchableChannel(true)),
        new InstantCommand(() -> SmartDashboard.putBoolean("Enable", false)));
  }

  @Override
  public void periodic() {
    //  SmartDashboard.putBoolean("Enable", false);
    Logger.recordOutput("activate Limelight ran", activateLimelight().isScheduled());
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      //  poseBuffer.addSample(inputs[i].poseTimeStamp,
      // RobotContainer.drive.getPoseAtTimeStamp(inputs[i].poseTimeStamp));
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    // Loop over cameras

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Logger.recordOutput("Debug Vision/tagCount", observation.tagCount());
        // Logger.recordOutput("Debug Vision/ambiguity", observation.ambiguity());
        // Logger.recordOutput(
        //     "Debug Vision/ambiguity above maxAmbiguity", observation.ambiguity() > maxAmbiguity);
        // Logger.recordOutput("Debug Vision/poseZ", observation.pose().getZ());
        // Logger.recordOutput(
        //     "Debug Vision/poseZ above maxZError", observation.pose().getZ() > maxZError);
        // Logger.recordOutput("Debug Vision/poseX", observation.pose().getX());
        // Logger.recordOutput(
        //     "Debug Vision/poseXWithinField",
        //     observation.pose().getX() > aprilTagLayout.getFieldLength());
        // Logger.recordOutput("Debug Vision/poseY", observation.pose().getY());
        // Logger.recordOutput(
        //     "Debug Vision/poseYWithinField",
        //     observation.pose().getY() > aprilTagLayout.getFieldWidth());
        Logger.recordOutput("Debug Vision/rejectPose", rejectPose);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
          //  poseBuffer.addSample(observation.timestamp(), observation.pose().toPose2d());
          // drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        } else if (observation.type() == PoseObservationType.MEGATAG_1) {
          linearStdDev = 0.5 * stdDevFactor;
          angularStdDev = 4.3 * stdDevFactor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Logger.recordOutput("Debug Vision/averageTagDistance", observation.averageTagDistance());
        // Logger.recordOutput("Debug Vision/stdDevFactor", stdDevFactor);
        // Logger.recordOutput("Debug Vision/linearStdDev", linearStdDev);
        // Logger.recordOutput("Debug Vision/angularStdDev", angularStdDev);
        // Logger.recordOutput("Debug Vision/angularStdDevBaseline", angularStdDevBaseline);
        // Logger.recordOutput(
        //     "Debug Vision/angularStdDevMegatag2Factor", angularStdDevMegatag2Factor);

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision Summary/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision Summary/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision Summary/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision Summary/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    // Logger.recordOutput(
    //     "Vision Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    // Logger.recordOutput(
    //     "Vision Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    // Logger.recordOutput(
    //     "Vision Summary/RobotPosesAccepted",
    //     allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    // Logger.recordOutput(
    //     "Vision Summary/RobotPosesRejected",
    //     allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  // rejecting old positions

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
