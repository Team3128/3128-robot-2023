package frc.team3128.common.hardware.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper for the WPILib XboxController class. 
 * To adjust for specific controller, change array for button names.
 * @author Arav Chadha
 */

public class NAR_XboxController extends XboxController {

    private String buttonNames[] = {
        "A",
        "B",
        "X",
        "Y",
        "LeftBumper",
        "RightBumper",
        "Back",
        "Start",
        "LeftStick",
        "RightStick"
    };

    private double leftXDeadband = 0.1;
    private double leftYDeadband = 0.1;
    private double rightXDeadband = 0.1;
    private double rightYDeadband = 0.1;
    
    private HashMap<String, Trigger> buttons;
    private Trigger[] povButtons;

    public NAR_XboxController(int port) {
        super(port);
        buttons = new HashMap<String, Trigger>();
        povButtons = new Trigger[8];
        for (int i = 0; i < 10; i++) {
            int n = i + 1;
            buttons.put(buttonNames[i], new Trigger (() -> getRawButton(n)));
        }   
        for (int i = 0; i < 8; i++) {
            int n = i;
            povButtons[i] = new Trigger (() -> getPOV() == n * 45);
        }
        buttons.put("RightTrigger", new Trigger(()-> getRightTriggerAxis() >= 0.5));
        buttons.put("LeftTrigger", new Trigger(()-> getLeftTriggerAxis() >= 0.5));
        buttons.put("LeftPosY", new Trigger(()-> getLeftY() >= 0.5));
        buttons.put("LeftNegY", new Trigger(()-> getLeftY() <= -0.5));
        buttons.put("RightPosY", new Trigger(()-> getRightY() >= 0.5));
        buttons.put("RightNegY", new Trigger(()-> getRightY() <= -0.5));
    }

    public Trigger getButton(String buttonName) {
        return buttons.get(buttonName);
    }

    public Trigger getLeftTrigger() {
        return new Trigger (() -> getLeftTriggerAxis() >= 0.5);
    }

    public Trigger getRightTrigger() {
        return new Trigger (() -> getRightTriggerAxis() >= 0.5);
    }

    public Trigger getPOVButton(int i) {
        return povButtons[i];
    }

    public Trigger getUpPOVButton() {
        return getPOVButton(0);
    }

    public Trigger getDownPOVButton() {
        return getPOVButton(4);
    }

    public Trigger getLeftPOVButton() {
        return getPOVButton(6);
    }

    public Trigger getRightPOVButton() {
        return getPOVButton(2);
    }

    public void startVibrate() {
        setRumble(RumbleType.kBothRumble, 0.8);
    }

    public void stopVibrate() {
        setRumble(RumbleType.kBothRumble, 0);
    }

     @Override
    public double getRightX() {
        return Math.abs(super.getRightX()) > rightXDeadband ? super.getRightX():0;
    }

    @Override
    public double getRightY() {
        return Math.abs(super.getRightY()) > rightYDeadband ? -super.getRightY():0;
    }

    @Override
    public double getLeftX() {
        return Math.abs(super.getLeftX()) > leftXDeadband ? super.getLeftX():0;
    }

    @Override
    public double getLeftY() {
        return Math.abs(super.getLeftY()) > leftYDeadband ? -super.getLeftY():0;
    }
}