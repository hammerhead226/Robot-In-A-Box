// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToStowAuto extends SequentialCommandGroup {
  /** Creates a new GoToStowAuto. */
  public GoToStowAuto(Elevator elevator, ScoralArm scoralArm, ScoralRollers scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG - 6, 10),
        scoralRollers.stopCommand(),
        new WaitUntilCommand(() -> scoralArm.atGoal(10)),
        new SetElevatorTarget(elevator, ElevatorConstants.STOW_SETPOINT_INCH, 15),
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG, 2));
  }
}
