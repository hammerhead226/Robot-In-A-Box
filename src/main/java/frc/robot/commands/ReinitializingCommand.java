package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class ReinitializingCommand extends Command {
  Command command;
  Supplier<Command> commandSupplier;

  public ReinitializingCommand(Supplier<Command> commandSupplier, Subsystem... requirements) {
    this.commandSupplier = commandSupplier;
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command = commandSupplier.get();
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
