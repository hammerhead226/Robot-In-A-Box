package frc.robot.subsystems.elevator;

// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorVis {
  private final String elevatorKey;
  private final LoggedMechanism2d elevatorPanel;
  private final LoggedMechanismRoot2d elevatorRoot;
  private final LoggedMechanismLigament2d elevatorMecha;

  public ElevatorVis(String elevatorKey, Color color) {
    this.elevatorKey = elevatorKey;
    this.elevatorPanel = new LoggedMechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.elevatorRoot = elevatorPanel.getRoot("elevator", 50, 0);
    this.elevatorMecha =
        elevatorRoot.append(
            new LoggedMechanismLigament2d("elevator", 2, 0, 10, new Color8Bit(color)));

    Logger.recordOutput("ElevatorVis/mechanism2d/" + elevatorKey, this.elevatorPanel);
  }
  // Updates the position of the root and the extedning length of the elevator
  public void update(/*double position,*/ double length) {
    // Sets the extending length of the elevator
    // elevatorRoot.setPosition(50, position);
    elevatorMecha.setLength(length);
    // elevatorRoot.setPosition(50, position);
    Logger.recordOutput("ElevatorVis/mechanism2d/" + elevatorKey, this.elevatorPanel);
  }
}
