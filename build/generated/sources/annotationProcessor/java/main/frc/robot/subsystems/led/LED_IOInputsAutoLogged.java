package frc.robot.subsystems.led;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LED_IOInputsAutoLogged extends LED_IO.LED_IOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LedState", ledState);
    table.put("CurrentAmps", currentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    ledState = table.get("LedState", ledState);
    currentAmps = table.get("CurrentAmps", currentAmps);
  }

  public LED_IOInputsAutoLogged clone() {
    LED_IOInputsAutoLogged copy = new LED_IOInputsAutoLogged();
    copy.ledState = this.ledState;
    copy.currentAmps = this.currentAmps;
    return copy;
  }
}
