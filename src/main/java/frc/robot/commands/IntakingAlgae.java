// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants;
<<<<<<< HEAD
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.CoralScorerDefinedPositions;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
=======
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakingAlgae extends Command {
  private final Elevator elevator;
  private final ClimberFeeder clFeeder;
  private final ClimberArm clArm;
  private final LED led;

  /** Creates a new IntakingAlgae. */
  public IntakingAlgae(Elevator elevator, ClimberFeeder clFeeder, ClimberArm clArm, LED led) {
    this.elevator = elevator;
    this.clFeeder = clFeeder;
    this.clArm = clArm;
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, clFeeder, clArm, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setExtenderGoal(12);
    arm.setArmGoal(CoralScorerArmConstants.INTAKE_SETPOINT_DEG);
    algaeIntake.runVelocity(AlgaeScorerFlywheelConstants.FLYWHEEL_VELOCITY_DEGPERSEC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_GREEN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stop();
    arm.setArmGoal(CoralScorerArmConstants.STOW_SETPOINT_DEG);
    elevator.setExtenderGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clFeeder.seesAlgae() == AlgaeState.CURRENT;
  }
}
