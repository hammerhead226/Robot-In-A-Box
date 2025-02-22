package frc.robot.subsystems.scoral;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVis {
  private final String key;
  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismRoot2d root2;
  private final LoggedMechanismLigament2d mecha2;
  private final LoggedMechanismLigament2d mecha;

  public ArmVis(String key, Color color) {

    this.key = key;
    this.panel = new LoggedMechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.root = panel.getRoot("mechanism", 50, 0.2);
    this.mecha =
        root.append(new LoggedMechanismLigament2d("arms", 0.5, 0, 10, new Color8Bit(color)));

    this.root2 = panel.getRoot("mechanism2", 50, -0.2);

    this.mecha2 =
        root2.append(new LoggedMechanismLigament2d("arms2", 0.5, 0, 10, new Color8Bit(color)));
    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
  }

  public void update(double angle) {
    // mecha.setLength(position);
    // root.setPosition(50, position);
    mecha.setAngle(angle);
    mecha2.setAngle(-angle);
    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
  }

  public void updateVertical(double position) {
    root.setPosition(50.1, position);
    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
  }

  public void updateLength(double length) {
    mecha.setLength(length);
    mecha2.setLength(length);
    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
  }
}
