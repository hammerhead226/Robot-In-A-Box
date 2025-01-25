package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class KeyboardInputs {
    private final Joystick keyboard;
    public KeyboardInputs(int port) {
        keyboard = new Joystick(port);
    }

    public JoystickButton getZButton() {
        return new JoystickButton(keyboard, 1);
    }

    public JoystickButton getXButton() {
        return new JoystickButton(keyboard, 2);
    }

    public JoystickButton getCButton() {
        return new JoystickButton(keyboard, 3);
    }

    public JoystickButton getVButton() {
        return new JoystickButton(keyboard, 4);
    }
}
