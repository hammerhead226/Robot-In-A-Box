package frc.robot.subsystems.scoral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class ScoralSensorCANRangeIO implements ScoralSensorIO {
  private final CANrange sensor;

  private StatusSignal<Distance> distanceMeters;

  public ScoralSensorCANRangeIO(int id) {
    sensor = new CANrange(id);

    distanceMeters = sensor.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(100, distanceMeters);
  }

  public void updateInputs(ScoralSensorIOInputs inputs) {
    BaseStatusSignal.refreshAll(distanceMeters);
    inputs.connected = sensor.isConnected();
    inputs.distanceInches = Units.metersToInches(distanceMeters.getValueAsDouble());
  }
}
