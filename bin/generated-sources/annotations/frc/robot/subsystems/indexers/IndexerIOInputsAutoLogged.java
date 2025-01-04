package frc.robot.subsystems.indexers;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndexerIOInputsAutoLogged extends IndexerIO.IndexerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRads", positionRads);
    table.put("VelocityRadsPerSec", velocityRadsPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRads = table.get("PositionRads", positionRads);
    velocityRadsPerSec = table.get("VelocityRadsPerSec", velocityRadsPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
  }

  public IndexerIOInputsAutoLogged clone() {
    IndexerIOInputsAutoLogged copy = new IndexerIOInputsAutoLogged();
    copy.positionRads = this.positionRads;
    copy.velocityRadsPerSec = this.velocityRadsPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    return copy;
  }
}
