// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgaeFromGroundCoral extends SequentialCommandGroup {
  /** Creates a new IntakeAlgaeFromGroundCoral. */
  private final ScoralArm scoralArm;

  private final Elevator elevator;
  private final ScoralRollers scoralRollers;

  public IntakeAlgaeFromGroundCoral(
      ScoralArm scoralArm, Elevator elevator, ScoralRollers scoralRollers) {
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;

    addCommands(
        new InstantCommand(() -> scoralRollers.runVolts(-2)),
        new SetElevatorTarget(elevator, ElevatorConstants.GROUND_CORAL_ALGAE_SETPOINT_INCHES, 2),
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.GROUND_CORAL_ALGAE_SETPOINT_DEG, 2));
  }
}
