package frc.robot.subsystems.scoral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class DistanceSensorCANRangeIO implements DistanceSensorIO {
  private final CANrange sensor;

  private StatusSignal<Distance> distanceMeters;

  public DistanceSensorCANRangeIO(int id, String CANBUS) {
    sensor = new CANrange(id, CANBUS);

    distanceMeters = sensor.getDistance();

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.FovParams.FOVRangeX = 13;
    config.FovParams.FOVRangeY = 13;

    sensor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(100, distanceMeters);
  }

  public void updateInputs(DistanceSensorIOInputs inputs) {
    BaseStatusSignal.refreshAll(distanceMeters);
    inputs.connected = sensor.isConnected();
    inputs.distanceInches = Units.metersToInches(distanceMeters.getValueAsDouble());
  }
}
