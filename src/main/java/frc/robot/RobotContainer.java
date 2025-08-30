package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.motor.Motor;
import frc.robot.subsystems.motor.MotorIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Motor motor;

  // Controller
  private final CommandXboxController testController = new CommandXboxController(0);
  // private final Joystick joystikc = new Joystick(0);
  // private final JoystickButton btn = new JoystickButton(joystikc, 4);
  // private final KeyboardInputs keyboard = new KeyboardInputs(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    motor = new Motor(new MotorIOTalonFX(11));
    // switch (SimConstants.currentMode) {
    //   case REAL:
    //     motor = new Motor(new MotorIOTalonFX(11)); // Test Motor CAN ID: 11.
    //     break;
    //   case SIM:
    //     motor = new Motor(new MotorIOTalonFX(11)); // Test Motor CAN ID: 11.
    //     break;

    //   default:
    //     motor = new Motor(new MotorIOTalonFX(11)); // Test Motor CAN ID: 11.
    //     break;
    // }
    testBindings();
  }

  private void testBindings() {
    testController.leftBumper().onTrue(new InstantCommand(() -> motor.runVelocity(10)));
  }

  public Motor getMotor() {
    return motor;
  }
}
