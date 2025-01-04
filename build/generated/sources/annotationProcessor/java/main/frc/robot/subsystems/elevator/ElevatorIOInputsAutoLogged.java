package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorPosition", elevatorPosition);
    table.put("ElevatorVelocity", elevatorVelocity);
    table.put("CurrentAmps", currentAmps);
    table.put("AppliedVolts", appliedVolts);
    table.put("PositionSetpoint", positionSetpoint);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorPosition = table.get("ElevatorPosition", elevatorPosition);
    elevatorVelocity = table.get("ElevatorVelocity", elevatorVelocity);
    currentAmps = table.get("CurrentAmps", currentAmps);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.elevatorPosition = this.elevatorPosition;
    copy.elevatorVelocity = this.elevatorVelocity;
    copy.currentAmps = this.currentAmps;
    copy.appliedVolts = this.appliedVolts;
    copy.positionSetpoint = this.positionSetpoint;
    return copy;
  }
}
