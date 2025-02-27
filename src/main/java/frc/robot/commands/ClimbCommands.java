// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCommands extends SequentialCommandGroup {
  ClimberArm climberArm;
  Winch winch;
  ScoralArm scoralArm;
  /** Creates a new ClimbCommands. */
  ScoralRollers scoralRollers;

  public ClimbCommands(
      ScoralArm scoralArm, ClimberArm climberArm, Winch winch, ScoralRollers scoralRollers) {
    this.climberArm = climberArm;
    this.winch = winch;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> climberArm.setVoltage(2)));
    // new WaitUntilCommand(() -> climberArm.hasReachedGoal(80)),
    // new InstantCommand(() -> climberArm.armStop()));
  }
}
