package frc.robot.subsystems.pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("VelocityDegsPerSec", velocityDegsPerSec);
    table.put("PositionDegs", positionDegs);
    table.put("CurrentAmps", currentAmps);
    table.put("AppliedVolts", appliedVolts);
    table.put("PositionSetpointDegs", positionSetpointDegs);
    table.put("GyroConnected", gyroConnected);
    table.put("Pitch", pitch);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityDegsPerSec = table.get("VelocityDegsPerSec", velocityDegsPerSec);
    positionDegs = table.get("PositionDegs", positionDegs);
    currentAmps = table.get("CurrentAmps", currentAmps);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    positionSetpointDegs = table.get("PositionSetpointDegs", positionSetpointDegs);
    gyroConnected = table.get("GyroConnected", gyroConnected);
    pitch = table.get("Pitch", pitch);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.velocityDegsPerSec = this.velocityDegsPerSec;
    copy.positionDegs = this.positionDegs;
    copy.currentAmps = this.currentAmps;
    copy.appliedVolts = this.appliedVolts;
    copy.positionSetpointDegs = this.positionSetpointDegs;
    copy.gyroConnected = this.gyroConnected;
    copy.pitch = this.pitch;
    return copy;
  }
}
