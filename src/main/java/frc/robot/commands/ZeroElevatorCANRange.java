// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroElevatorCANRange extends Command {
  /** Creates a new ZeroElevatorCANRange. */
  private Elevator elevator;
  public ZeroElevatorCANRange(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setVoltage(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
    elevator.zeroElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getCanRangeDistanceInches() <= SubsystemConstants.ElevatorConstants.CANRAGE_ZERO_SETPOINT_INCHES;
  }
}
