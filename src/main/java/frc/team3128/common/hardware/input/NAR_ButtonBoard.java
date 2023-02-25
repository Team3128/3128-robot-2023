package frc.team3128.common.hardware.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 *  Wrapper for the WPILib Joystick class. Works with device that can be recognized as joystick as driverstation.
 * @author Peter Ma, Arav Chadha
 */
public class NAR_ButtonBoard {

    private Joystick stick;

    private Trigger[] buttons;

    public NAR_ButtonBoard(int deviceNumber) {
        buttons = new Trigger[16];
        stick = new Joystick(deviceNumber);

        // 2023 btnboard has 12 buttons & 4 axis direction

        for (int i = 0; i < 12; i++) {
            int buttonId = i;
            buttons[buttonId] = new Trigger(() -> stick.getRawButton(buttonId + 1)); 
        }

        buttons[12] = new Trigger(() -> stick.getX() == -1.0);
        buttons[13] = new Trigger(() -> stick.getX() == 1.0);
        buttons[14] = new Trigger(() -> stick.getY() == 1.0);
        buttons[15] = new Trigger(() -> stick.getY() == -1.0);
            
    }

    public Trigger getButton(int i) {
        return buttons[i-1];
    }

    public Trigger getPosXButton() {
        return buttons[12];
    }

    public Trigger getNegXButton() {
        return buttons[13];
    }

    public Trigger getPosYButton() {
        return buttons[14];
    }

    public Trigger getNegYButton() {
        return buttons[15];
    }
}