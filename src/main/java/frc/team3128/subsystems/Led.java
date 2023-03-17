package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.LedConstants;
import frc.team3128.Constants.LedConstants.*;

public class Led extends SubsystemBase{

    private static Led instance;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public Led(){
        initLEDs();
    }
    
    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }
        return instance;
    }
    
    public void initLEDs() {
        led = new AddressableLED(LedConstants.PORT);

        ledBuffer = new AddressableLEDBuffer(LedConstants.LENGTH);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
        
    }
    
    //general color methods : use variables
    public void setRGB(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values
            ledBuffer.setRGB(i, r, g, b);
        }
         
        led.setData(ledBuffer);
    }
    
    public void setHSV(int h, int s, int v) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values
            ledBuffer.setHSV(i, h, s, v);
        }
         
        led.setData(ledBuffer);
    }

    //custom color methods : use constants
    public void setAllianceColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            setHSV(Blue.HUE, Blue.SATURATION, Blue.VALUE);
        }
        else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            setHSV(Red.HUE, Red.SATURATION, Red.VALUE);
        }
    }

    public void setColorYellow() {
        setHSV(Yellow.HUE, Yellow.SATURATION, Yellow.VALUE);
    }

    public void setColorPurple() {
        setHSV(Purple.HUE, Purple.SATURATION, Purple.VALUE);
    }

    public void setAutoColor() {
        setHSV(Green.HUE, Green.SATURATION, Green.VALUE);
    }

    public void setPickupColor(boolean cone) {
        if (cone) {
            setColorYellow();
        } else {
            setColorPurple();
        }
    }
    

}
