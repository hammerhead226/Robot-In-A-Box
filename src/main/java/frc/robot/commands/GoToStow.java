// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToStow extends SequentialCommandGroup {
  private final Elevator elevator;

  private final ScoralArm scoralArm;
  private final ScoralRollers scoralRollers;
  /** Creates a new GoToStowL4. */
  public GoToStow(Elevator m_elevator, ScoralArm m_scoralArm, ScoralRollers m_scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = m_elevator;
    this.scoralArm = m_scoralArm;
    this.scoralRollers = m_scoralRollers;

    addCommands(
        scoralRollers.stopCommand(),
        scoralArm.setArmTarget(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG + 4, 10),
        new WaitUntilCommand(() -> scoralArm.atGoal(10)),
        elevator.setElevatorTarget(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH, 2),
        new WaitCommand(0.3),
        scoralArm.setArmTarget(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG, 2));
  }
}
