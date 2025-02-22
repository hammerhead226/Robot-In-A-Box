// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToStowLowerLevels extends SequentialCommandGroup {
  /** Creates a new goToStow. */
  private final Elevator elevator;

  private final ScoralArm scoralArm;
  private final ScoralRollers scoralRollers;

  public GoToStowLowerLevels(
      Elevator m_elevator, ScoralArm m_scoralArm, ScoralRollers m_scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = m_elevator;
    this.scoralArm = m_scoralArm;
    this.scoralRollers = m_scoralRollers;

    addCommands(
        scoralRollers.stopCommand(),
        scoralArm.setArmTarget(
            SubsystemConstants.CoralScorerConstants.ScoralArmConstants.STOW_SETPOINT_DEG, 2),
        new WaitUntilCommand(() -> scoralArm.atGoal(2)),
        elevator.setFirstStageTarget(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH, 2));
  }
}
