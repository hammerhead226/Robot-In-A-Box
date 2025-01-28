package frc.robot.subsystems.coralscorer;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVis {
  private final String armKey;
  private final LoggedMechanism2d armPanel;
  private final LoggedMechanismRoot2d armRoot;
  private final LoggedMechanismLigament2d armMecha;

  public ArmVis(String key, Color color) {
    this.armKey = key;
    this.armPanel = new LoggedMechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.armRoot = armPanel.getRoot("elevator", 50, 0);
    this.armMecha =
        armRoot.append(new LoggedMechanismLigament2d("elevator", 2, 0, 10, new Color8Bit(color)));
  }

  public void update(double position) {
    armMecha.setLength(position);
    armRoot.setPosition(50, position);
    Logger.recordOutput("PivotVis/mechanism2d/" + armKey, this.armPanel);
  }
}
