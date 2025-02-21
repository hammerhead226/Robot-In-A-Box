package frc.robot.subsystems.coralscorer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.SubsystemConstants;

public class CoralSensorCANRangeIO implements CoralSensorIO {
  private final CANrange sensor;

  private StatusSignal<Distance> distanceMeters;

  public CoralSensorCANRangeIO(int id) {
    sensor = new CANrange(id, SubsystemConstants.CANIVORE_ID_STRING);

    distanceMeters = sensor.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(100, distanceMeters);
  }

  public void updateInputs(CoralSensorIOInputs inputs) {
    inputs.connected = sensor.isConnected();
    inputs.distanceInches = Units.metersToInches(distanceMeters.getValueAsDouble());
  }
}
